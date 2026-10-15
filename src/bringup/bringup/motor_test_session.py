"""Private terminal bridge for one authenticated motor-test session."""
import fcntl
import os
from pathlib import Path
import pty
import select
import signal
import socket
import stat
import subprocess
import termios
import tty


def controlling_terminal():
    os.setsid()
    fcntl.ioctl(0, termios.TIOCSCTTY, 0)


def stop_child(process):
    if process is None:
        return
    # ROS launch propagates SIGINT to its driver before returning.
    for sig, wait in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=wait)
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def serve(directory, commands, owner, lifetime_fd=0):
    """Only fixed, precomputed commands are accepted; sockets carry terminal bytes."""
    directory = Path(directory)
    info = directory.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != owner or stat.S_IMODE(info.st_mode) != 0o700:
        raise ValueError('Session directory must be private and owned by the invoking user')
    listeners, channels = {}, {}

    def close_channel(sock):
        master, process = channels.pop(sock)
        # Keep the PTY open while ROS handles its normal shutdown signal.
        stop_child(process)
        os.close(master)
        sock.close()

    try:
        for role in commands:
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            path = directory / f'{role}.sock'
            listener.bind(str(path))
            os.chmod(path, 0o600)
            os.chown(path, owner, -1)
            listener.listen(1)
            listeners[listener] = role
        (directory / 'ready').write_text('ready\n')
        while True:
            readers = [lifetime_fd, *listeners, *channels, *(v[0] for v in channels.values())]
            ready, _, _ = select.select(readers, [], [], .2)
            if lifetime_fd in ready:
                if not os.read(lifetime_fd, 1):
                    break
            for listener in list(listeners):
                if listener not in ready:
                    continue
                sock, _ = listener.accept()
                # Filesystem permissions and peer identity both restrict attachment.
                import struct
                _, peer_uid, _ = struct.unpack('3i', sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                if peer_uid != owner:
                    sock.close()
                    continue
                role = listeners.pop(listener)
                listener.close()
                master, slave = pty.openpty()
                try:
                    process = subprocess.Popen(commands[role], stdin=slave, stdout=slave,
                                               stderr=slave, preexec_fn=controlling_terminal)
                except BaseException:
                    os.close(master); sock.close()
                    raise
                finally:
                    os.close(slave)
                sock.settimeout(1)
                channels[sock] = (master, process)
            for sock, (master, process) in list(channels.items()):
                try:
                    if sock in ready:
                        data = sock.recv(4096)
                        if not data:
                            close_channel(sock)
                            continue
                        # Small terminal input writes; preserve all bytes.
                        while data:
                            data = data[os.write(master, data):]
                    if master in ready:
                        data = os.read(master, 4096)
                        if not data:
                            close_channel(sock)
                            continue
                        sock.sendall(data)
                except (OSError, TimeoutError):
                    close_channel(sock)
    finally:
        for sock in list(channels):
            close_channel(sock)
        for listener in listeners:
            listener.close()
        for role in commands:
            (directory / f'{role}.sock').unlink(missing_ok=True)
        (directory / 'ready').unlink(missing_ok=True)


def attach(path):
    """Forward a pane's terminal to the already authenticated fixed process."""
    original = termios.tcgetattr(0) if os.isatty(0) else None
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.connect(str(path))
        try:
            if original is not None:
                tty.setraw(0)
            while True:
                ready, _, _ = select.select([0, sock], [], [])
                if sock in ready:
                    data = sock.recv(4096)
                    if not data:
                        break
                    while data:
                        data = data[os.write(1, data):]
                if 0 in ready:
                    data = os.read(0, 4096)
                    if not data:
                        break
                    sock.sendall(data)
        finally:
            if original is not None:
                termios.tcsetattr(0, termios.TCSADRAIN, original)
    print('\nProcess exited; this pane will not restart it automatically.', flush=True)
