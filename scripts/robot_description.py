"""Shared MJCF robot metadata. Importers own geometry; runtimes own algorithms."""
import xml.etree.ElementTree as ET


def metadata(scene, profile):
    custom = scene.find('custom')
    if custom is None:
        custom = ET.SubElement(scene, 'custom')
    def text(name, value):
        if not value:
            return
        ET.SubElement(custom, 'text', name=name, data=value if isinstance(value, str) else ' '.join(value))
    text('robot_id', profile['id'])
    text('robot_root', profile['root_body'])
    text('robot_tool', profile.get('tool_site', 'arm_tool'))
    text('robot_joints', profile['joint_names'])
    for name, values in [('robot_velocity_limits', profile['velocity_limits']),
                         ('robot_planning_home', profile['planning_home']),
                         ('robot_posture_joint', [profile.get('posture_joint', -1)])]:
        ET.SubElement(custom, 'numeric', name=name, data=' '.join(format(x, '.17g') for x in values))
    if profile.get('recovery_joint_order'):
        ET.SubElement(custom, 'numeric', name='robot_recovery_joint_order', data=' '.join(str(x) for x in profile['recovery_joint_order']))
    scenario = profile.get('scenario', {})
    for field in ('core_pose', 'assembly_translation'):
        if field in scenario:
            ET.SubElement(custom, 'numeric', name='robot_'+field, data=' '.join(format(x, '.17g') for x in scenario[field]))
    for field in ('assembly_yaw', 'target_degrees', 'enforce_core_pose_limits'):
        if field in scenario:
            ET.SubElement(custom, 'numeric', name='robot_'+field, data=format(float(scenario[field]), '.17g'))
    return custom


def collision_metadata(scene, arm_geoms, mount_bodies, grasp_geoms=()):
    custom = scene.find('custom')
    for name, values in [('robot_collision_geoms', arm_geoms), ('robot_mount_bodies', mount_bodies),
                         ('robot_grasp_geoms', grasp_geoms)]:
        old = custom.find(f"text[@name='{name}']")
        if old is not None:
            custom.remove(old)
        if values:
            ET.SubElement(custom, 'text', name=name, data=' '.join(values))


def rm26_profile():
    return dict(id='rm26', root_body='j0_link',
                joint_names=['j0', 'j1', 'j2', 'j3', 'j4', 'end_differential_controller/j5', 'end_differential_controller/j6'],
                velocity_limits=[1.0]*7, posture_joint=6, recovery_joint_order=[6,3,0,4,1,2,5],
                planning_home=[-.62770373654521094, 2.3297993305507192, 1.0289871622237392,
                               1.2715734368717495, -1.747192543846495, -.81681197466957878, 1.9772845777886507])
