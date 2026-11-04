"""Stable visual colors for the integrated robot; never changes contact geometry."""
import xml.etree.ElementTree as ET

# Keyed by rigid body, not traversal order: regeneration keeps colors stable.
BODY_COLORS = {
    'body': ('Chassis', '#87939d'),
    'b2_suspension_left_2': ('Left suspension', '#b9a38c'),
    'b2_suspension_right_3': ('Right suspension', '#8daaa5'),
    'suspension_3': ('Front suspension', '#a499b4'),
    'freedrive_1': ('Left front swivel', '#b89fa9'),
    'b3_wheel_linkage_6': ('Right front swivel', '#a6b397'),
    'linkage_wheel_1': ('Wheel 1', '#c5b39b'),
    'linkage_wheel_2': ('Wheel 2', '#bf9c94'),
    'linkage_wheel_3': ('Wheel 3', '#94b3b7'),
    'linkage_wheel_4': ('Wheel 4', '#92a5bb'),
    'linkage_wheel_5': ('Wheel 5', '#baa8bf'),
    'linkage_wheel_6': ('Wheel 6', '#b2bca0'),
    'b2_arm_d1': ('Arm D1 shoulder', '#8b9fb4'),
    'b3_arm_d2': ('Arm D2 upper link', '#bca18b'),
    'l3_arm_linkage_d3': ('Arm D3 crank', '#98ad96'),
    'b4_1_arm_rod_d3': ('D3 passive rod', '#c3b88f'),
    'b4_2_arm_d3': ('D3 passive bearing / forearm', '#aa9db7'),
    'l4_arm_linkage_d4': ('Arm D4 wrist', '#bd9da3'),
    'l5_arm_linkage_d5': ('Arm D5 wrist', '#8fada6'),
    'l6_arm_linkage_d6': ('Arm D6 tool', '#9db5c2'),
}


def rgba(hex_color):
    return ' '.join(f'{int(hex_color[i:i+2], 16) / 255:.6f}' for i in (1, 3, 5)) + ' 1'


def apply_body_colors(root):
    """Color group-2 meshes only; wheel textures still reveal rotation.

    Can be applied repeatedly to integrated XML or the fixed-chassis arm preview.
    Unknown bodies fail explicitly so new CAD links cannot silently share a color.
    """
    bodies = [(b, [g for g in b.findall('geom') if g.get('group') == '2'])
              for b in root.findall('.//worldbody//body')]
    unknown = [b.get('name') for b, geoms in bodies if geoms and b.get('name') not in BODY_COLORS]
    if unknown:
        raise ValueError(f'Add visual colors for new bodies: {unknown}')
    asset = root.find('asset')
    if asset is None:
        asset = ET.SubElement(root, 'asset')
    if asset.find("texture[@name='wheel_checker']") is None:
        ET.SubElement(asset, 'texture', name='wheel_checker', type='cube', builtin='checker',
                      width='256', height='256', rgb1='0.12 0.12 0.12', rgb2='1 1 1')
    # Gentle checker contrast keeps wheel rotation visible without black/white bands.
    texture = asset.find("texture[@name='wheel_checker']")
    texture.set('rgb1', '0.5 0.5 0.5')
    texture.set('rgb2', '0.9 0.9 0.9')
    for material in list(asset.findall('material')):
        if material.get('name') == 'wheel_checker_material' or material.get('name', '').startswith('body_color_'):
            asset.remove(material)
    for body, geoms in bodies:
        if not geoms:
            continue
        name = body.get('name')
        color = rgba(BODY_COLORS[name][1])
        wheel = name.startswith('linkage_wheel_')
        if wheel:
            ET.SubElement(asset, 'material', name='body_color_' + name,
                          texture='wheel_checker', rgba=color, specular='0.1', shininess='0.1')
        for geom in geoms:
            geom.set('rgba', color)
            if wheel:
                geom.set('material', 'body_color_' + name)
            else:
                geom.attrib.pop('material', None)
