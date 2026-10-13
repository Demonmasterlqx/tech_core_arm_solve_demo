"""Robot-independent technology-core task geometry and hollow carried unit.

Only the caller supplies a robot/tool attachment. Geometric dimensions and
contact surface names are shared by every robot and the planning validator.
"""
from copy import deepcopy
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def task_components(output_directory):
    core = ET.parse(ROOT/'models/tech_core.xml').getroot()
    asset = ET.Element('asset')
    for child in core.find('asset'):
        if child.get('name', '').startswith('core1_'):
            continue
        part = deepcopy(child)
        if part.tag == 'mesh' and 'file' in part.attrib:
            part.set('file', os.path.relpath(ROOT/'models'/part.get('file'), output_directory))
        asset.append(part)
    base = deepcopy(core.find('./worldbody/body[@name="core0_base"]'))
    base.set('pos', '.39 0 .35')
    base.set('quat', '0 0 0 1')
    carriage = base.find('.//body[@name="core0_carriage"]')
    carried = carriage.find('./body[@name="core0_energy_unit"]')
    carriage.remove(carried)
    for joint in list(carried.findall('joint')):
        carried.remove(joint)
    carried.set('name', 'carried_unit')
    for element in carried.iter():
        if 'name' in element.attrib:
            element.set('name', element.get('name').replace('core0_unit_', 'carried_unit_').replace('core0_grip', 'carried_unit_grip').replace('core0_opening', 'carried_unit_opening'))
    ET.SubElement(carried, 'site', name='planning_tcp', pos='0 0 0', quat='.5 .5 .5 .5', size='.003', rgba='0 0 0 0')
    ET.SubElement(carried, 'site', name='carried_unit_closed_cap', pos='.15 0 0', size='.003', rgba='0 0 0 0')
    # Cover the source's 25--35 mm collar. Circumscribed outer walls avoid
    # inscribed-polygon undercoverage; retain the audited hollow bore inside.
    for name, old_length, new_length, outer in [('collision_unit_flange',.025,.035,.040),('collision_unit_tube',.075,.065,.0325)]:
        mesh = asset.find(f'mesh[@name="{name}"]')
        vertices = list(map(float, mesh.get('vertex').split()))
        for k in range(0, len(vertices), 3):
            if abs(vertices[k]-old_length)<1e-8:
                vertices[k] = new_length
            if abs(math.hypot(vertices[k+1],vertices[k+2])-outer)<1e-8:
                vertices[k+1] /= math.cos(math.pi/48)
                vertices[k+2] /= math.cos(math.pi/48)
        mesh.set('vertex', ' '.join(f'{x:.14g}' for x in vertices))
    for geom in carried.findall('geom'):
        if geom.get('name','').startswith('carried_unit_tube_'):
            geom.set('pos','.035 0 0')
    vertices = [tuple(map(float, line.split()[1:4])) for line in (ROOT/'models/meshes/core/shell.obj').read_text().splitlines() if line.startswith('v ')]
    lo = [min(v[k] for v in vertices) for k in range(3)]
    hi = [max(v[k] for v in vertices) for k in range(3)]
    ET.SubElement(base, 'geom', name='collision_core_shell', type='box',
                  pos=' '.join(str((a+b)/2) for a,b in zip(lo,hi)),
                  size=' '.join(str((b-a)/2) for a,b in zip(lo,hi)), rgba='0 0 0 0', group='3')
    block = ET.Element('geom', name='collision_block_fixture', type='box', pos='0 0 -10', size='.08 .08 .08', rgba='.8 .15 .1 .35', group='3')
    return asset, base, carried, block, lo, hi
