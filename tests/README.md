# Motor tests

Follow the [Ubuntu build guide](../docs/ENCOS_WORKSPACE.md#build-on-ubuntu);
`./scripts/build.sh` builds and runs all tests with CTest.

- `encos_protocol_test.c`: query/position/velocity codec vectors and malformed frames.
- `test_python_motors.py`: simulation, independent commands, deadlines, validation and cleanup.
- `test_differential_drive.py`: simulation-derived wheel signs, chassis arcs, RPM
  saturation, motor ordering, timestamp validation and keyboard dead-man stopping.
- `test_ros_chassis.py`: real ROS topics through the existing simulated motor
  driver, including upstream publisher loss; skips without ROS 2. Source the ROS
  environment and run on an isolated ROS domain with no hardware or other publishers.
- `test_native_driver.py`: real C pthread and ctypes ABI against a fake SOEM bridge;
  transport/feedback failures, motor faults, per-motor leases, ownership, shutdown
  and differential forward/left/right/stop commands through all four PDO slots.

The fake library has no raw-socket transport and cannot access motor hardware.
Native integration cases skip unless `ENCOS_TEST_LIBRARY` names that library;
CTest sets it automatically. The normal loader uses `libencos_driver.so`, not
`libencos_driver_test.so`.

Portable Python-only tests:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -v
```

On PowerShell:

```powershell
$env:PYTHONPATH = 'src'
python -m unittest discover -s tests -p 'test_*.py' -v
```

These checks do not validate physical stopping or bridge feedback freshness.
