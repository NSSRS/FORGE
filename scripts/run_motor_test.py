#!/usr/bin/env python3
"""Open the complete motor test in a separate four-pane Terminator window."""
import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
ROLES = ('driver', 'keyboard', 'relay', 'viewer')
TITLES = ('1. Motor driver', '2. W/S/A/D keyboard', '3. Read-only feedback relay', '4. MuJoCo viewer')


def command(role, simulate, domain, authenticated=False):
    profile = str(ROOT / 'src/forge_motor_control/config/chassis_bench.yaml')
    if role == 'viewer':
        return [str(ROOT / 'scripts/run_motor_viewer.sh')]
    if role == 'driver':
        args = ['ros2', 'launch', 'forge_motor_control', 'chassis.launch.py',
                f'config:={profile}', f'simulate:={str(simulate).lower()}',
                f'execute:={str(not simulate).lower()}',
                f'mapping_confirmed:={str(not simulate).lower()}']
    elif role == 'keyboard':
        args = ['ros2', 'run', 'forge_motor_control', 'forge_keyboard_teleop',
                '--ros-args', '--params-file', profile]
    else:
        args = ['ros2', 'run', 'forge_motor_control', 'forge_motor_state_relay']
    script = (f'cd {shlex.quote(str(ROOT))}\n'
              f'source {shlex.quote(str(ROOT / "scripts/ros2_env.sh"))}\n'
              f'export ROS_DOMAIN_ID={domain}\nexec {shlex.join(args)}')
    return ([] if simulate or authenticated else ['sudo']) + ['bash', '-lc', script]


def layout(simulate, domain, session=None):
    result = {
        'window': {'type': 'Window', 'parent': '', 'size': [1200, 800]},
        'columns': {'type': 'HPaned', 'parent': 'window', 'position': 600},
        'left': {'type': 'VPaned', 'parent': 'columns', 'order': 0, 'position': 400},
        'right': {'type': 'VPaned', 'parent': 'columns', 'order': 1, 'position': 400},
    }
    for index, (role, title) in enumerate(zip(ROLES, TITLES)):
        args = [sys.executable, str(Path(__file__).resolve()), '--pane', role, '--domain', str(domain)]
        if session and role != 'viewer':
            args += ['--session', str(session)]
        if simulate:
            args.append('--simulate')
        result[role] = dict(type='Terminal', parent='left' if index < 2 else 'right',
                            order=index % 2, profile='forge', title=title,
                            command=shlex.join(args))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulate', action='store_true', help='Run without hardware or sudo (domain 42)')
    parser.add_argument('--domain', type=int, help='ROS domain (hardware 44, simulation 42)')
    parser.add_argument('--dry-run', action='store_true', help='Show commands without launching anything')
    parser.add_argument('--pane', choices=ROLES, help=argparse.SUPPRESS)
    parser.add_argument('--session', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--serve', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    domain = args.domain if args.domain is not None else (42 if args.simulate else 44)
    if not 0 <= domain <= 101:
        parser.error('choose a ROS domain from 0 through 101')
    if args.serve:
        if os.geteuid() != 0 or 'SUDO_UID' not in os.environ:
            parser.error('the authenticated session must be started by the launcher through sudo')
        from motor_test_session import serve
        serve(args.serve, {role: command(role, False, domain, authenticated=True)
                           for role in ROLES if role != 'viewer'}, int(os.environ['SUDO_UID']))
        return
    if args.dry_run:
        if not args.simulate:
            print('One sudo authentication in launcher; driver/keyboard/relay run in its private session.')
        for role in ROLES:
            print(f'{role}: {shlex.join(command(role, args.simulate, domain, authenticated=True))}')
        return
    if args.pane:
        print(f'FORGE {args.pane} — {"SIMULATION" if args.simulate else "HARDWARE"}', flush=True)
        if args.pane == 'keyboard':
            print('Wait for the driver to report Ready before holding W/S/A/D. Space/X stops.', flush=True)
        if args.pane == 'viewer':
            print('The MuJoCo graphical window opens separately; closing it does not stop motors.', flush=True)
        try:
            if args.session and args.pane != 'viewer':
                from motor_test_session import attach
                attach(args.session / f'{args.pane}.sock')
                return
            if not args.simulate and args.pane != 'viewer':
                parser.error('hardware panes require an authenticated launcher session')
            result = subprocess.run(command(args.pane, args.simulate, domain), cwd=ROOT)
            print(f'Process exited with status {result.returncode}.', flush=True)
        except KeyboardInterrupt:
            print('\nPane stopped.', flush=True)
        return
    if os.geteuid() == 0:
        parser.error('run this launcher as your normal desktop user, without sudo')
    if not shutil.which('terminator'):
        parser.error('install Terminator first: sudo apt install terminator')
    if not (ROOT / 'build/motor_viewer/venv/bin/python').exists():
        parser.error('run ./scripts/setup_motor_viewer.sh first')
    if not (ROOT / 'install/ros2/setup.bash').exists():
        parser.error('build the ROS package first; see docs/MOTOR_MIRROR.md')
    # Do not add a second EtherCAT owner or command publisher to an existing run.
    processes = subprocess.run(['pgrep', '-f', r'(^|[ /])(forge_motor_driver|forge_keyboard_teleop|forge_motor_state_relay)( |$)|scripts/mujoco_motor_viewer\.py'],
                               capture_output=True, text=True)
    if processes.returncode == 0:
        parser.error('a motor driver, keyboard, relay or viewer is already running; stop the old test first')
    from configobj import ConfigObj
    if not args.simulate:
        print('Authenticate once for the driver, keyboard and relay.', flush=True)
        subprocess.run(['sudo', '-v'], check=True)
    with tempfile.TemporaryDirectory(prefix='forge-motor-test-') as folder:
        session = Path(folder)
        server = None
        try:
            if not args.simulate:
                server = subprocess.Popen(
                    ['sudo', '-n', '/usr/bin/python3', str(Path(__file__).resolve()),
                     '--serve', str(session), '--domain', str(domain)], stdin=subprocess.PIPE)
                deadline = time.monotonic() + 10
                while not (session / 'ready').exists():
                    if server.poll() is not None:
                        raise RuntimeError('Authenticated session failed to start')
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Authenticated session startup timed out')
                    time.sleep(.05)
            config = ConfigObj()
            config['global_config'] = {'suppress_multiple_term_dialog': False}
            config['profiles'] = {'forge': {'use_system_font': False, 'font': 'Monospace 10',
                                            'scrollback_lines': 5000, 'exit_action': 'hold'}}
            config['layouts'] = {'forge': layout(args.simulate, domain,
                                                None if args.simulate else session)}
            config['plugins'] = {}
            path = session / 'terminator.conf'
            config.filename = str(path)
            config.write()
            print('Opening four panes. No additional sudo prompts.', flush=True)
            print('The MuJoCo GUI opens separately. Focus the keyboard pane to drive.', flush=True)
            subprocess.run(['terminator', '--no-dbus', '--config', str(path), '--layout', 'forge'],
                           check=True)
        finally:
            if server is not None:
                # Pipe closure ends this session, including any remaining ROS children.
                server.stdin.close()
                server.wait(timeout=25)


if __name__ == '__main__':
    main()
