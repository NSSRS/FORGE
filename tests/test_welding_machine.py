"""Exercise the welder client against local Modbus TCP and RTU devices."""

from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import importlib.util
import os
import select
import socketserver
import struct
import threading
import unittest

import source_paths
from imu.protocol import crc
from welding_machine import WelderClient, WelderDeviceError, WelderError
from welding_machine.cli import main


MODBUS_AVAILABLE = importlib.util.find_spec("pymodbus") is not None
SERIAL_AVAILABLE = importlib.util.find_spec("serial") is not None


@contextmanager
def tcp_device(replies):
    requests = []

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            while header := self.rfile.read(7):
                transaction, protocol, length, unit = struct.unpack(">HHHB", header)
                request = self.rfile.read(length - 1)
                requests.append((protocol, unit, request))
                response = next(replies)
                if response is None:
                    return
                self.wfile.write(struct.pack(">HHHB", transaction, 0, len(response) + 1, unit)
                                 + response)

    with socketserver.TCPServer(("127.0.0.1", 0), Handler) as server:
        worker = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
        worker.start()
        try:
            yield server.server_address[1], requests
        finally:
            server.shutdown()
            worker.join()


def rtu_frame(data):
    return data + struct.pack("<H", crc(data))


@unittest.skipUnless(MODBUS_AVAILABLE, "install the welder extra to test Modbus")
class WelderTests(unittest.TestCase):
    def test_tcp_status_tables_and_zero_based_addressing(self):
        replies = iter((
            bytes.fromhex("03 04 12 34 fe dc"),
            bytes.fromhex("04 02 00 2a"),
            bytes.fromhex("01 01 05"),
            bytes.fromhex("02 02 01 01"),
        ))
        with tcp_device(replies) as (port, requests):
            with WelderClient.tcp("127.0.0.1", port=port, unit=31) as client:
                self.assertEqual(client.read("holding-registers", 0x12, count=2), (0x1234, 0xfedc))
                self.assertEqual(client.read("input-registers", 0xffff), (42,))
                self.assertEqual(client.read("coils", 0, count=3), (True, False, True))
                self.assertEqual(client.read("discrete-inputs", 10, count=9),
                                 (True,) + (False,) * 7 + (True,))
            self.assertEqual(requests, [
                (0, 31, bytes.fromhex(message)) for message in (
                    "03 00 12 00 02", "04 ff ff 00 01", "01 00 00 00 03", "02 00 0a 00 09",
                )
            ])

    def test_tcp_device_exception_keeps_connection_available(self):
        replies = iter((bytes.fromhex("83 02"), bytes.fromhex("03 02 00 2a")))
        with tcp_device(replies) as (port, requests):
            with WelderClient.tcp("127.0.0.1", port=port, unit=31) as client:
                with self.assertRaises(WelderDeviceError) as caught:
                    client.read("holding-registers", 0xffff)
                self.assertEqual((caught.exception.unit, caught.exception.function, caught.exception.code),
                                 (31, 3, 2))
                self.assertEqual(client.read("holding-registers", 0), (42,))
            self.assertEqual(len(requests), 2)

    def test_incomplete_tcp_reply_and_disconnect_do_not_retry(self):
        for reply in (bytes.fromhex("03 02 00 2a"), None):
            with self.subTest(reply=reply), tcp_device(iter((reply,))) as (port, requests):
                with WelderClient.tcp("127.0.0.1", port=port, unit=1, timeout=0.2) as client:
                    with self.assertRaises(WelderError):
                        client.read("holding-registers", 0, count=2)
                self.assertEqual(requests, [(0, 1, bytes.fromhex("03 00 00 00 02"))])

    def test_invalid_requests_are_rejected_before_io(self):
        with tcp_device(iter(())) as (port, requests):
            with WelderClient.tcp("127.0.0.1", port=port, unit=1) as client:
                for table, address, count in (
                    ("write-coil", 0, 1),
                    ("holding-registers", True, 1),
                    ("input-registers", 0xffff, 2),
                    ("holding-registers", 0, 126),
                    ("coils", 0, 2001),
                ):
                    with self.subTest(table=table, address=address, count=count):
                        with self.assertRaises(ValueError):
                            client.read(table, address, count=count)
            self.assertEqual(requests, [])
        with self.assertRaisesRegex(ValueError, "Unit ID"):
            WelderClient.rtu("unused", unit=0, baudrate=9600, parity="N", stopbits=1)

    def test_cli_returns_raw_values_for_documented_block(self):
        from contextlib import redirect_stdout
        from io import StringIO
        import json

        with tcp_device(iter((bytes.fromhex("04 02 00 2a"),))) as (port, requests):
            with redirect_stdout(StringIO()) as output:
                status = main([
                    "tcp", "--host", "127.0.0.1", "--tcp-port", str(port), "--unit", "19",
                    "--table", "input-registers", "--address", "0x20",
                ])
            self.assertEqual(status, 0)
            self.assertEqual(json.loads(output.getvalue())["values"], [42])
            self.assertEqual(requests, [(0, 19, bytes.fromhex("04 00 20 00 01"))])

    @unittest.skipUnless(os.name == "posix" and SERIAL_AVAILABLE, "requires a POSIX PTY and pyserial")
    def test_rtu_request_and_crc_over_pseudo_terminal(self):
        import pty

        for corrupt in (False, True):
            with self.subTest(corrupt=corrupt):
                master, slave = pty.openpty()
                try:
                    with ThreadPoolExecutor(max_workers=1) as pool:
                        with WelderClient.rtu(
                            os.ttyname(slave), unit=7, baudrate=19200, parity="N",
                            stopbits=1, timeout=0.25,
                        ) as client:
                            result = pool.submit(client.read, "holding-registers", 0x20, count=2)
                            request = b""
                            while len(request) < 8:
                                self.assertTrue(select.select([master], [], [], 2)[0])
                                request += os.read(master, 8 - len(request))
                            self.assertEqual(request, rtu_frame(bytes.fromhex("07 03 00 20 00 02")))
                            reply = rtu_frame(bytes.fromhex("07 03 04 12 34 fe dc"))
                            if corrupt:
                                os.write(master, reply[:-1] + bytes([reply[-1] ^ 1]))
                                with self.assertRaises(WelderError):
                                    result.result(timeout=3)
                                self.assertFalse(select.select([master], [], [], 0.05)[0])
                            else:
                                os.write(master, reply)
                                self.assertEqual(result.result(timeout=3), (0x1234, 0xfedc))
                finally:
                    os.close(slave)
                    os.close(master)
