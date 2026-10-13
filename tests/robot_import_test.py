#!/usr/bin/env python3
"""An external two-axis package proves registration needs no C++ edits/rebuild."""
import argparse
import json
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--control', type=Path, required=True)
    args = parser.parse_args()
    for entry in json.loads((ROOT/'reference/mec_arm/manifest.json').read_text())['files']:
        assert hashlib.sha256((ROOT/entry['local_path']).read_bytes()).hexdigest()==entry['sha256'], entry['local_path']
    with tempfile.TemporaryDirectory(prefix='external-robot-') as temporary:
        directory = Path(temporary)
        (directory/'source.urdf').write_text('''<robot name="external">
          <link name="base"/><link name="slider"><visual><geometry><box size=".1 .1 .1"/></geometry></visual></link>
          <link name="wrist"><visual><geometry><sphere radius=".02"/></geometry></visual></link><link name="tool"/>
          <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/>
            <origin xyz="0 0 .3" rpy=".1 .2 .3"/><axis xyz="1 0 0"/><limit lower="-.5" upper=".5" effort="1" velocity=".2"/></joint>
          <joint name="turn" type="revolute"><parent link="slider"/><child link="wrist"/>
            <origin xyz=".2 0 0" rpy="0 0 .1"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity=".4"/></joint>
          <joint name="tcp" type="fixed"><parent link="wrist"/><child link="tool"/><origin xyz=".1 0 0"/></joint>
        </robot>''')
        profile = dict(id='external',urdf='source.urdf',root_body='base',tool_link='tool',
                       joint_names=['turn','slide'],home=[.3,.1],planning_home=[.3,.1],velocity_limits=[.4,.2])
        config = directory/'robot.json'
        config.write_text(json.dumps(profile))
        command = [sys.executable,str(ROOT/'scripts/import_robot.py'),'--config',str(config)]
        subprocess.run(command,check=True,cwd=directory,capture_output=True)
        original = (directory/'scene.xml').read_bytes()
        subprocess.run(command,check=True,cwd=directory,capture_output=True)
        assert (directory/'scene.xml').read_bytes()==original, 'Importer is not reproducible'
        scene = ET.parse(directory/'scene.xml').getroot()
        assert scene.find('./keyframe/key').get('qpos').split()==['0.10000000000000001','0.29999999999999999']
        run = subprocess.run([str(args.control.resolve()),'--robot',str(directory),'--headless','--duration','.1',
                              '--velocity','.1,.02'],check=True,text=True,capture_output=True,cwd=directory)
        assert 'Joint order: turn slide' in run.stdout, run.stdout
        final = json.loads(next(line.split(': ',1)[1] for line in run.stdout.splitlines() if line.startswith('State final: ')))
        assert max(abs(a-b) for a,b in zip(final['q'],[.31,.102]))<1e-10, final
        # Runtime loaders also reject tampered metadata, independently of importer.
        for field, value in [('robot_joints','turn turn'), ('robot_tool','missing_tool')]:
            broken = ET.fromstring(original)
            broken.find(f'./custom/text[@name="{field}"]').set('data',value)
            ET.ElementTree(broken).write(directory/'broken.xml')
            bad = subprocess.run([str(args.control.resolve()),'--model',str(directory/'broken.xml'),'--headless'],capture_output=True)
            assert bad.returncode!=0, field
        # Missing/duplicate coordinates and undeclared moving joints fail early.
        profile['joint_names']=['turn','turn'];config.write_text(json.dumps(profile))
        assert subprocess.run(command,capture_output=True).returncode!=0
        profile['joint_names']=['turn'];profile['home']=[.3];profile['planning_home']=[.3];profile['velocity_limits']=[.4]
        config.write_text(json.dumps(profile))
        assert subprocess.run(command,capture_output=True).returncode!=0
        print('External package import, mixed joint types, reordered commands and rejection checks passed')


if __name__=='__main__':
    main()
