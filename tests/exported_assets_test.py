#!/usr/bin/env python3
"""Check the pinned CAD delivery and the complete generated model dependency set."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--planner', type=Path)
args = parser.parse_args()

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'reference/exported_arms'
manifest = json.loads((source / 'checksums.json').read_text())


def check(path, expected):
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise AssertionError('Changed model dependency: '+str(path))


for entry in manifest['files']:
    path = source / entry['path']
    assert path.stat().st_size == entry['bytes'], path
    check(path, entry['sha256'])
assert json.loads((source / 'finished').read_text())['status'] == 'finished'
for robot in ('hkust_eng', 'pie_eng', 'zhuimeng_single', 'zhuimeng_dual'):
    directory = ROOT / 'models' / robot
    geometry = json.loads((directory / 'geometry_report.json').read_text())
    report = json.loads((directory / 'import_report.json').read_text())
    for name, expected in {**geometry['generated_sha256'], **report['files_sha256']}.items():
        check(directory / name, expected)
    for name in ('scene.xml', 'planning.xml'):
        for asset in ET.parse(directory / name).findall('./asset/mesh'):
            if asset.get('file') is None:
                assert asset.get('vertex'), 'Mesh has neither an external file nor inline vertices'
                continue
            path = (directory / asset.get('file')).resolve()
            assert path.is_file() and path.is_relative_to(ROOT), path
    print(robot, 'source and generated assets verified')
print(len(manifest['files']), 'original export files are byte-identical')

if args.planner:
    directory = ROOT / 'models/hkust_eng'
    template = ET.parse(directory / 'planning.xml').getroot()
    for asset in template.find('asset'):
        if asset.get('file'):
            asset.set('file', str((directory / asset.get('file')).resolve()))
    cases = [
        ('text', 'robot_compound_joint_bodies', 'base_link yaw_link', 'exactly two'),
        ('text', 'robot_compound_joint_bodies', 'base_link tool_link', 'exactly two'),
        ('numeric', 'robot_demo_start_at_preset', '2', 'Invalid demo start pose flag'),
    ]
    with tempfile.TemporaryDirectory(prefix='exported-metadata-') as temporary:
        path = Path(temporary) / 'invalid.xml'
        for kind, name, value, expected in cases:
            tree = deepcopy(template)
            tree.find(f"custom/{kind}[@name='{name}']").set('data', value)
            ET.ElementTree(tree).write(path, encoding='utf-8')
            run = subprocess.run([str(args.planner.resolve()), '--model', str(path), '--list'],
                                 text=True, capture_output=True, timeout=20)
            assert run.returncode != 0 and expected in run.stderr, (name, value, run.stdout, run.stderr)
    print('Invalid compound-joint declarations and startup flags rejected')
