"""Prepare the commissioned camera link without changing unrelated interfaces."""
import ipaddress
import json
import subprocess
import shutil

CAMERA_PORTS = ('enp86s0', 'enx000ec8a22028')


def network_plan(interfaces, interface, host_address, camera_ip):
    host = ipaddress.IPv4Interface(host_address)
    camera = ipaddress.IPv4Address(camera_ip)
    if camera not in host.network or camera == host.ip:
        raise ValueError('Camera and host must have different addresses in the same subnet')
    by_name = {entry['ifname']: entry for entry in interfaces}
    if interface == 'auto':
        candidates = [name for name in CAMERA_PORTS if name in by_name
                      and 'LOWER_UP' in by_name[name].get('flags', [])]
        if len(candidates) != 1:
            raise ValueError('Need exactly one connected camera port; connect the camera or '
                             'set camera_interface:=enp86s0 (or enx000ec8a22028)')
        interface = candidates[0]
    if interface not in CAMERA_PORTS or interface not in by_name:
        raise ValueError('Camera interface must be one of the two commissioned Ethernet ports')
    selected = by_name[interface]
    if 'LOWER_UP' not in selected.get('flags', []):
        raise ValueError(f'{interface}: no Ethernet carrier; check camera power and cable')
    commands = []
    configured = False
    for name, entry in by_name.items():
        for address in entry.get('addr_info', []):
            if address.get('family') != 'inet':
                continue
            existing = ipaddress.IPv4Interface(f"{address['local']}/{address['prefixlen']}")
            if name == interface:
                if existing == host:
                    configured = True
                else:
                    raise ValueError(f'{interface} already has IPv4 address {existing}; '
                                     'refusing to change a port used by another network')
            elif existing.network.overlaps(host.network):
                # Only remove our exact temporary address from a disconnected,
                # previously commissioned camera port. Never flush addresses.
                if (name in CAMERA_PORTS and existing == host
                        and 'LOWER_UP' not in entry.get('flags', [])):
                    commands.append(['ip', 'address', 'del', str(host), 'dev', name])
                else:
                    raise ValueError(f'Camera subnet conflicts with {name}: {existing}')
    if not configured:
        commands.append(['ip', 'address', 'add', str(host), 'dev', interface])
    return interface, commands


def activate_camera_profile(interface, host_address):
    """Activate a dedicated saved profile, without changing the motor profile.

    Autoconnect is off: camera-only launch selects this profile after link and
    subnet checks. The desktop's NetworkManager policy permits setup without sudo.
    """
    name = f'FORGE-camera-{interface}'
    result = subprocess.run(
        ['nmcli', '-g', 'connection.interface-name,ipv4.addresses',
         'connection', 'show', name], capture_output=True, text=True, timeout=10)
    if result.returncode == 0:
        if result.stdout.strip().splitlines() != [interface, host_address]:
            raise RuntimeError(f'{name} does not match the expected camera port/address')
    else:
        subprocess.run([
            'nmcli', 'connection', 'add', 'type', 'ethernet', 'con-name', name,
            'ifname', interface, 'connection.autoconnect', 'no',
            'ipv4.method', 'manual', 'ipv4.addresses', host_address,
            'ipv4.never-default', 'yes', 'ipv6.method', 'disabled',
        ], check=True, timeout=15)
    subprocess.run(['nmcli', '--wait', '10', 'connection', 'up', name,
                    'ifname', interface], check=True, timeout=15)


def prepare_network(interface='auto', host_address='192.168.1.100/24',
                    camera_ip='192.168.1.10'):
    result = subprocess.run(['ip', '-j', 'address', 'show'], check=True,
                            capture_output=True, text=True, timeout=5)
    interface, commands = network_plan(json.loads(result.stdout), interface,
                                       host_address, camera_ip)
    for command in commands:
        print(f'Preparing camera network: {" ".join(command)}', flush=True)
        if command[:3] == ['ip', 'address', 'add'] and shutil.which('nmcli'):
            activate_camera_profile(interface, host_address)
            continue
        result = subprocess.run(['sudo', *command])
        if result.returncode:
            raise RuntimeError('Camera network setup needs sudo authentication in this terminal; '
                               'the camera driver has not been started')
    print(f'Camera port selected: {interface}', flush=True)
    print(f'Checking camera {camera_ip} on {interface}...', flush=True)
    result = subprocess.run(['ping', '-n', '-c', '3', '-W', '1', '-I', interface, camera_ip],
                            timeout=6)
    if result.returncode:
        raise RuntimeError('Camera did not answer ping; check power, cable and address. '
                           'The camera driver has not been started')
