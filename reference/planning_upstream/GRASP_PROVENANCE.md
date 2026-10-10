# Energy-unit grasp TCP evidence

Pinned upstream: `6edc630a1a14a3904b42d5a360e9020562eefc99`.
This is a local audit note; referenced source files are exact upstream bytes.

No calibrated energy-unit-to-tool transform was found in the reviewed v3 URDF,
`axis_free_rotate_ik`, `handle_arm`, `rm26_energy_unit_pap`, v3 automatic-pick/place
configuration, v3 custom-controller configuration, or connecting-rod gripper
controller. This scope-bounded absence does not prove none exists in unpublished
CAD, hardware calibration, or other branches.

## What the sources establish

- Existing `reference/rm26_arm/source.urdf:464–472` defines empty `ee_link` and
  fixed `ee_joint` from `j6_link`, translation `(0, .06161, -.15266)` m and
  `Rx(pi/2)`. It does not label this as energy-unit closed cap, opening, or grasp
  centroid. No object attachment link is present.
- `src/RM/RMController/core/application/rm26_energy_unit_pap/Readme.md` and its
  `src/rm26_energy_unit_pap.cpp` execute pre-recorded joint trajectories and
  publish Boolean open/close commands. They do not estimate or define an object
  pose, Cartesian grasp transform, or TF for an attached energy unit.
- `core/test/rm26_version3_engineering_model_moveit/config/auto_pap_energy_unit.yaml`
  contains trajectory-file slots, gripper topics, waits, and seven-joint recorded
  approach/getout poses. These joint states alone do not identify the held
  object's transform without a known object pose at the corresponding instant.
- `core/test/rm26_version3_engineering_model_moveit/config/ros2_controllers.yaml:220–243`
  configures motor `end_joint`, left/right connecting-rod joints, open `-.26` rad,
  close `3.0` rad, position threshold `.3` rad, one reach frame, and enabled
  no-PID mode with open effort `-1.5`, close effort `+1.5`.
- `core/rm_connecting_rod_gripper_controller/src/control_part.cpp:29–32` actually
  exports left and right joint positions equal to motor position, without a
  ratio or offset. Their URDF axes have opposite signs. The mapping is not merely
  README prose.
- `gripper_core.hpp` treats positive motion as closing. At q=0, the distance from
  configured open position is `.26 < .3` rad, so this can satisfy the source
  OPENED condition; fixing source gripper joints at zero is not evidence of a
  physically closed grasp.

## Limits of the evidence

The URDF left/right limits start at `-.1` rad, whereas the runtime open target is
`-.26` rad. This discrepancy prevents treating every configuration number as a
calibrated CAD pose. The configured close value is a controller target/state
threshold, not a supplied object contact pose. Contact may stop the jaws before
that value, and the snapshot alone does not identify the object's actual
location or dimensions at contact.

Consequently, a chosen rigid end-face or jaw fixture transform must stay an
explicit local simulation assumption. It may be checked against source CAD and
known unit geometry, but cannot be described as an upstream TCP calibration.
Changing the fixed jaw angle away from zero is also an explicit adaptation;
carry it in the grasp manifest and collision geometry contract.
