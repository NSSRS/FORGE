# Motor tests

Follow the [Ubuntu build guide](../docs/ENCOS_WORKSPACE.md#build-on-ubuntu);
`./scripts/build.sh` builds and runs all tests with CTest.

- `encos_protocol_test.c`: query/position/velocity codec vectors and malformed frames.
- `test_python_motors.py`: simulation, independent commands, deadlines, validation and cleanup.
- `test_native_driver.py`: real C pthread and ctypes ABI against a fake SOEM bridge;
  transport/feedback failures, motor faults, per-motor leases, ownership and shutdown.

The fake library has no raw-socket transport and cannot access motor hardware.
Native integration cases skip unless `ENCOS_TEST_LIBRARY` names that library;
CTest sets it automatically. The normal loader uses `libencos_driver.so`, not
`libencos_driver_test.so`.

Portable Python-only tests:

```bash
PYTHONPATH=python python3 -m unittest discover -s tests -p 'test_*.py' -v
```

On PowerShell:

```powershell
$env:PYTHONPATH = 'python'
python -m unittest discover -s tests -p 'test_*.py' -v
```

These checks do not validate physical stopping or bridge feedback freshness.
