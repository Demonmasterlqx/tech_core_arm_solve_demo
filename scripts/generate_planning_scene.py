#!/usr/bin/env python3
"""Generate a CPU kinematic planning scene; explicit geomDistance owns collisions.

No source asset is copied or modified. The carried unit is a declared rigid
two-jaw midsection grasp with explicit fixed auxiliary joint angles; no grasp dynamics.
"""
from pathlib import Path
from copy import deepcopy
import json
import struct
import hashlib
import math
import sys
from collections import defaultdict
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'models/planning'
ARM = ROOT / 'models/rm26_arm/scene.xml'
CORE = ROOT / 'models/tech_core.xml'


def component_meshes(name):
    """Weld STL vertices at 0.1 micrometre, then retain each connected solid.

    MuJoCo computes each component convex hull; the source visual is untouched.
    This removes convex bridges between disconnected pieces of a CAD assembly.
    """
    raw = (ROOT / 'models/rm26_arm/meshes' / (name + '.STL')).read_bytes()
    n = struct.unpack_from('<I', raw, 80)[0]
    parent, seen, triangles = list(range(n)), {}, []
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(n):
        tri = [struct.unpack_from('<3f', raw, 84+50*i+12+12*j) for j in range(3)]
        triangles.append(tri)
        for v in tri:
            key = tuple(round(x, 7) for x in v)
            if key in seen:
                parent[find(i)] = find(seen[key])
            else:
                seen[key] = i
    groups = defaultdict(list)
    for i in range(n):
        groups[find(i)].append(triangles[i])
    directory = OUT / 'collision'
    directory.mkdir(parents=True, exist_ok=True)
    result = []
    for index, faces in enumerate(sorted(groups.values(), key=lambda g: (-len(g), g[0]))):
        verts, ids, mapped = [], {}, []
        for face in faces:
            f = []
            for vertex in face:
                if vertex not in ids:
                    ids[vertex] = len(verts)+1
                    verts.append(vertex)
                f.append(ids[vertex])
            mapped.append(f)
        filename = f'{name}_part_{index}.obj'
        text = '# Connected component of source STL; no visual asset changes.\n'
        text += ''.join('v '+' '.join(f'{x:.10g}' for x in v)+'\n' for v in verts)
        text += ''.join('f '+' '.join(str(x) for x in f)+'\n' for f in mapped)
        (directory / filename).write_text(text)
        result.append((index, filename, len(faces)))
    return result


def regenerate_jaw_prisms():
    """Optional offline regeneration: numpy + shapely==2.2.0, one CPU worker.

    Fill only projected internal holes, partition the full outer silhouette,
    and extrude through the component's complete axial range. The retained
    union covers every source triangle projection and every source Z value.
    """
    import time
    import numpy as np
    from shapely.geometry import Polygon, MultiPoint
    from shapely.ops import unary_union
    from shapely import constrained_delaunay_triangles
    audit=OUT/'collision/jaw_prism_decomposition.json'
    old_audit=json.loads(audit.read_text()) if audit.exists() else {}
    start=time.perf_counter();out={'method':'source connected-component XY projection outer silhouette, convex partition, full Z extrusion','shapely_version':'2.2.0','buffer_m':1e-7,'sources':{}}
    for name in ['end_left_link','end_right_link']:
     components=component_meshes(name);entries=[]
     for ci,file,nt in components:
      vs=[];fs=[]
      for l in ((OUT/'collision')/file).read_text().splitlines():
       if l.startswith('v '):vs.append(list(map(float,l.split()[1:4])))
       elif l.startswith('f '):fs.append([int(i)-1 for i in l.split()[1:]])
      v=np.array(vs);tri=v[np.array(fs)];polys=[]
      for t in tri:
       p=Polygon(t[:,:2]);
       if p.area>1e-20:polys.append(p)
      shadow=unary_union(polys)
      filled=unary_union([Polygon(p.exterior) for p in ([shadow] if shadow.geom_type=='Polygon' else shadow.geoms)]).buffer(1e-7,quad_segs=1,join_style='mitre').simplify(1e-10,preserve_topology=True)
      ts=list(constrained_delaunay_triangles(filled).geoms)
      changed=True
      while changed:
       changed=False
       for i in range(len(ts)):
        for j in range(i+1,len(ts)):
         if ts[i].distance(ts[j])>1e-9:continue
         both=unary_union([ts[i],ts[j]]);h=both.convex_hull
         if h.area-both.area<=1e-15:
          ts[i]=h;ts.pop(j);changed=True;break
        if changed:break
      union=unary_union(ts)
      missing=shadow.difference(union).area
      max_vertex=max(union.distance(__import__('shapely').geometry.Point(*p)) for p in v[:,:2])
      print(name,'component',ci,'faces',nt,'convex',len(ts),'missing_area',missing,'vertex_outside',max_vertex,flush=True)
      assert missing<1e-16 and max_vertex<1e-9
      z0,z1=v[:,2].min()-1e-7,v[:,2].max()+1e-7
      for p in ts:
       xy=list(p.exterior.coords)[:-1];nv=len(xy);verts=[(x,y,z) for z in (z0,z1) for x,y in xy];faces=[]
       for k in range(1,nv-1):faces.extend([(0,k+1,k),(nv,nv+k,nv+k+1)])
       for k in range(nv):j=(k+1)%nv;faces.extend([(k,j,nv+j),(k,nv+j,nv+k)])
       k=len(entries);f=(OUT/'collision')/f'{name}_grasp_part_{k}.obj'
       f.write_text('# Conservative convex prism of source component projected silhouette.\n'+''.join('v '+' '.join(f'{x:.12g}' for x in row)+'\n' for row in verts)+''.join('f '+' '.join(str(x+1) for x in row)+'\n' for row in faces))
       entries.append({'file':f.name,'component':ci,'vertices':len(verts),'faces':len(faces),'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'projected_uncovered_source_area_m2':missing,'max_source_vertex_outside_m':max_vertex,'z_range_m':[z0,z1]})
     out['sources'][name]={'source_sha256':hashlib.sha256(((ROOT/'models/rm26_arm/meshes')/(name+'.STL')).read_bytes()).hexdigest(),'parts':entries}
    out['offline_seconds']=time.perf_counter()-start
    out['grip_fixed_pose']=old_audit.get('grip_fixed_pose',{})
    (OUT/'collision/jaw_prism_decomposition.json').write_text(json.dumps(out,indent=2)+'\n')
    print('total seconds',out['offline_seconds'])

    for name in ('end_left_link','end_right_link'):
     for file in (OUT/'collision').glob(name+'_part_*.obj'):file.unlink()


def generate():
    arm = ET.parse(ARM).getroot()
    core = ET.parse(CORE).getroot()
    arm.set('model', 'rm26_level3_kinematic_planning')
    compiler = arm.find('compiler')
    compiler.attrib.pop('meshdir', None)
    compiler.set('inertiafromgeom', 'false')
    for mesh in arm.findall('./asset/mesh'):
        mesh.set('file', '../rm26_arm/meshes/' + mesh.get('file'))
    asset = arm.find('asset')
    for child in core.find('asset'):
        if child.get('name', '').startswith('core1_'):
            continue
        part = deepcopy(child)
        if part.tag == 'mesh' and 'file' in part.attrib:
            part.set('file', '../' + part.get('file'))
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
    carried.set('pos', '0 -.015 -.010')
    carried.set('quat', '.3535533905932738 -.6123724356957945 -.3535533905932738 -.6123724356957945')
    for element in carried.iter():
        if 'name' in element.attrib:
            element.set('name', element.get('name').replace('core0_unit_', 'carried_unit_').replace('core0_grip', 'carried_unit_grip').replace('core0_opening', 'carried_unit_opening'))
    ET.SubElement(carried, 'site', name='planning_tcp', pos='0 0 0', quat='.5 .5 .5 .5', size='.003', rgba='0 0 0 0')
    ET.SubElement(carried, 'site', name='carried_unit_closed_cap', pos='.15 0 0', size='.003', rgba='0 0 0 0')
    # Fixed auxiliary jaw angles from source geometry, not simulated grasp.
    jaw_angle = 2.0504
    def qmul(a,b):
        w,x,y,z=a; W,X,Y,Z=b
        return [w*W-x*X-y*Y-z*Z,w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W]
    for name, sign in [('end_left_link',-1),('end_right_link',1)]:
        body=arm.find(f'.//body[@name="{name}"]')
        quat=qmul(list(map(float,body.get('quat').split())),[math.cos(jaw_angle/2),0,0,sign*math.sin(jaw_angle/2)])
        body.set('quat',' '.join(f'{x:.16g}' for x in quat))
    # Capture the 25--35 mm source collar; circumscribed outer wall avoids
    # inscribed-polygon undercoverage of the cylindrical exterior.
    for name, old_length, new_length, outer in [('collision_unit_flange',.025,.035,.040),('collision_unit_tube',.075,.065,.0325)]:
        mesh=asset.find(f'mesh[@name="{name}"]')
        vs=list(map(float,mesh.get('vertex').split()))
        for k in range(0,len(vs),3):
            if abs(vs[k]-old_length)<1e-8:vs[k]=new_length
            if abs(math.hypot(vs[k+1],vs[k+2])-outer)<1e-8:
                vs[k+1]/=math.cos(math.pi/48);vs[k+2]/=math.cos(math.pi/48)
        mesh.set('vertex',' '.join(f'{x:.14g}' for x in vs))
    for geom in carried.findall('geom'):
        if geom.get('name','').startswith('carried_unit_tube_'):geom.set('pos','.035 0 0')
    arm.find('.//body[@name="ee_link"]').append(carried)
    arm.find('worldbody').append(base)
    arm.find('./worldbody/geom[@name="ground_visual"]').set('pos','0 0 -.25')
    ET.SubElement(arm.find('worldbody'), 'geom', name='collision_block_fixture', type='box', pos='0 0 -10', size='.08 .08 .08', rgba='.8 .15 .1 .35', group='3')
    # Filled shell AABB is a conservative obstacle: it can reject routes through
    # source cavities, but never deliberately removes shell material.
    vertices = [tuple(map(float, line.split()[1:4])) for line in (ROOT / 'models/meshes/core/shell.obj').read_text().splitlines() if line.startswith('v ')]
    lo = [min(v[k] for v in vertices) for k in range(3)]
    hi = [max(v[k] for v in vertices) for k in range(3)]
    ET.SubElement(base, 'geom', name='collision_core_shell', type='box',
                  pos=' '.join(str((a+b)/2) for a,b in zip(lo,hi)),
                  size=' '.join(str((b-a)/2) for a,b in zip(lo,hi)),
                  rgba='0 0 0 0', group='3')
    arm.find('keyframe/key[@name="home"]').set('qpos', '-.62770373654521094 2.3297993305507192 1.0289871622237392 1.2715734368717495 -1.747192543846495 -.81681197466957878 1.9772845777886507 0 0 0')
    # All narrow-phase queries are explicit and independent of these masks.
    for geom in arm.iter('geom'):
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
    arm.insert(0, ET.Comment(' Generated by scripts/generate_planning_scene.py. Kinematic CPU planning model; all contacts require explicit geomDistance pair checking. '))
    decomposition = {}
    for name in ('j4_link', 'j6_link'):
        body = arm.find(f'.//body[@name="{name}"]')
        decomposition[name] = []
        for index, filename, triangles in component_meshes(name):
            mesh_name = f'planning_{name}_part_{index}'
            ET.SubElement(asset, 'mesh', name=mesh_name, file='collision/'+filename)
            geom_name = f'collision_arm_{name}_part_{index}'
            ET.SubElement(body, 'geom', name=geom_name, type='mesh', mesh=mesh_name,
                          contype='0', conaffinity='0', group='3', rgba='0 0 0 0')
            decomposition[name].append({'geom':geom_name,'mesh':filename,'source_triangles':triangles})
    jaw_manifest=json.loads((OUT/'collision/jaw_prism_decomposition.json').read_text())
    for name, source in jaw_manifest['sources'].items():
        original=ROOT/'models/rm26_arm/meshes'/(name+'.STL')
        if hashlib.sha256(original.read_bytes()).hexdigest()!=source['source_sha256']:
            raise ValueError('Source jaw mesh changed; regenerate collision decomposition')
        body=arm.find(f'.//body[@name="{name}"]')
        decomposition[name]=[]
        for index, part in enumerate(source['parts']):
            file=OUT/'collision'/part['file']
            if hashlib.sha256(file.read_bytes()).hexdigest()!=part['sha256']:
                raise ValueError('Jaw collision component checksum mismatch: '+str(file))
            mesh_name=f'planning_{name}_grasp_{index}'
            geom_name=f'collision_arm_{name}_part_{index}'
            if index in jaw_manifest.get('grip_fixed_pose',{}).get('near_contact_parts',{}).get(name,[]):geom_name += '_grip'
            ET.SubElement(asset,'mesh',name=mesh_name,file='collision/'+part['file'])
            ET.SubElement(body,'geom',name=geom_name,type='mesh',mesh=mesh_name,contype='0',conaffinity='0',group='3',rgba='0 0 0 0')
            decomposition[name].append({'geom':geom_name,'mesh':part['file'],'source_component':part['component']})
    arm_geoms = []
    def walk(body, cluster=None):
        if body.find('joint') is not None:
            cluster = body.get('name')
        for geom in body.findall('geom'):
            if geom.get('type') == 'mesh' and geom.get('name') not in ('j4_link_visual','j6_link_visual','end_left_link_visual','end_right_link_visual'):
                arm_geoms.append({'geom': geom.get('name'), 'rigid_cluster': cluster, 'body': body.get('name')})
        for child in body.findall('body'):
            if child.get('name') != 'carried_unit':
                walk(child, cluster)
    walk(arm.find('./worldbody/body[@name="j0_link"]'))
    # The rulebook peg proxy supersedes visual peg parts; the visual central
    # magnet protrudes 4.63 mm past nominal E and conflicts with a 100 mm stop.
    replaced = [g.get('name') for g in carriage.findall('geom') if any(s in g.get('name', '') for s in ('pillar_side_',))]
    moving = [g.get('name') for g in carriage.findall('geom') if g.get('type') == 'mesh' and g.get('name') not in replaced]
    unit = [g.get('name') for g in carried.findall('geom') if not g.get('name', '').startswith('carried_unit_visual_')]
    manifest = {
        'schema_version': 1, 'scene': 'models/planning/scene.xml',
        'collision_query': 'explicit mj_geomDistance; all contype/conaffinity masks are zero',
        'joint_order': [j.get('name') for j in arm.findall('.//worldbody//joint')],
        'planning_home_q': [-.62770373654521094,2.3297993305507192,1.0289871622237392,1.2715734368717495,-1.747192543846495,-.81681197466957878,1.9772845777886507],
        'T_world_from_assembly': {'translation_m': [.30, .2, -.25], 'quaternion_wxyz': [0,0,0,1]},
        'T_ee_from_unit': {'translation_m': [0,-.015,-.010], 'quaternion_wxyz': [.3535533905932738,-.6123724356957945,-.3535533905932738,-.6123724356957945]},
        'grasp_contract': 'pre-carried rigid two-jaw midsection grasp; source auxiliary jaws fixed at 2.0504 rad; no grasp dynamics or force validation',
        'jaw_angles_rad': {'gripper_controller/end_left_joint':jaw_angle,'gripper_controller/end_right_joint':jaw_angle},
        'planning_tcp': {'body':'carried_unit','position_local_m':[0,0,0],'quaternion_wxyz':[.5,.5,.5,.5],'meaning':'opening centre; local Z follows unit longitudinal +X'},
        'arm_mesh_convex_hulls': arm_geoms,
        'arm_component_decomposition': decomposition,
        'grasp_contact_candidates': [{'jaw_geom':part['geom'],'unit_geoms':['carried_unit_tube_*','carried_unit_iron'],'condition':'fixed pregrasp outer sidewall contact only; iron contact is exterior r=.0325 at x=.1.. .11, not internal stop face; penetration <=.0001 m'} for name in ('end_left_link','end_right_link') for part in decomposition[name] if part['geom'].endswith('_grip')],
        'jaw_decomposition_audit': 'models/planning/collision/jaw_prism_decomposition.json',
        'fixed_mount_contact_exemptions': [{'rigid_cluster':name,'geom':'mount_visual','reason':'fixed j0 mount / immediately adjoining j1 mount joint'} for name in ('j0_link','j1_link')],
        'adjacent_rigid_clusters': [['j'+str(i)+'_link','j'+str(i+1)+'_link'] for i in range(6)],
        'fixed_obstacles': ['ground_visual', 'mount_visual', 'collision_core_shell', 'collision_block_fixture'],
        'core_moving_mesh_convex_hulls': moving,
        'core_peg': ['core0_peg'], 'carried_unit_hollow_proxies': unit,
        'shell_aabb_local_m': {'min': lo, 'max': hi},
        'visual_peg_parts_replaced_by_rulebook_cylinder': replaced,
        'assembly_contact_candidates': [
            {'geom1':'core0_peg','geom2':'carried_unit_iron','condition':'axial endpoint s=.1 only; touching allowed; penetration <=.0001 m'},
            *[{'geom1':name,'geom2':'carried_unit_flange_*','condition':'opening face seated against mount x=-.1 only; touching allowed; penetration <=.0001 m'} for name in moving if any(s in name for s in ('pillar_base_', 'pillar_inner_ring_', 'pillar_L_light_'))]
        ],
        'recommended_external_clearance_m': .001,
        'recommended_assembly_penetration_tolerance_m': .0001,
        'limitations': [
            'Shell AABB and arm/moving mesh convex hulls are conservative and can yield false rejections.',
            'Nominal peg cylinder is r=.018 and x=[-.1,0]; visual central magnet protrusion is intentionally not collision geometry.',
            'Same rigid cluster and adjacent joint clusters need explicit audited self-contact exemptions; never skip all arm self-collision.',
            'Only explicitly tagged gripping jaw parts may contact carried_unit_tube or iron-cylinder outer sidewall; all other arm-unit pairs remain forbidden.',
            'Allowed assembly contact is phase and geometry specific, never a blanket unit-core exemption.',
            'Ground and mount are simple local scene obstacles; chassis and full field structure remain absent.'
        ]
    }
    OUT.mkdir(parents=True, exist_ok=True)
    ET.indent(arm, space='  ')
    ET.ElementTree(arm).write(OUT/'scene.xml', encoding='utf-8', xml_declaration=True)
    (OUT/'collision_manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({'scene':str(OUT/'scene.xml'),'joint_order':manifest['joint_order'],'arm_geoms':len(arm_geoms),'moving_core_geoms':len(moving),'unit_proxies':len(unit)}))

if __name__ == '__main__':
    if '--regenerate-jaws' in sys.argv:
        regenerate_jaw_prisms()
    generate()
