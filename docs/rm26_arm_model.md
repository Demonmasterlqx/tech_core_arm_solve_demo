# RM26 fixed-base arm model

`models/rm26_arm/scene.xml` imports the actual RM26 version 3 arm CAD meshes and kinematic chain from [HKUSTGZ-ROBOMASTER-PNX/UpperComputerControlOpenSourceDoc](https://github.com/HKUSTGZ-ROBOMASTER-PNX/UpperComputerControlOpenSourceDoc/tree/6edc630a1a14a3904b42d5a360e9020562eefc99/src/RM/RMController/core/models/rm26_version3_engineering_model), pinned to commit `6edc630a1a14a3904b42d5a360e9020562eefc99`.

The model contains **seven independent arm joints**, with no chassis, wheels, supports, storage geometry, chassis degrees of freedom, or actuators. The ground and small mount cylinder are visual reference geometry only. The complete subtree rooted at source joint `j0` is retained. Its camera, end mechanism, and gripper meshes and frames remain attached; the auxiliary revolute joints `end_joint`, `gripper_controller/end_left_joint`, and `gripper_controller/end_right_joint` are fixed at zero. The source's `end_link` has no active visual element; the unused `end_link.STL` is therefore not imported. The two empty camera optical-offset links are preserved as source frames.

## Coordinates and joint contract

Lengths are metres; angles are radians. World coordinates equal the source `base_link` coordinates, with identity alignment. Although the chassis is omitted, the original `j0` mounting transform `[-0.180999999999978, 0, 0.117999999999997]` m is retained. Every descendant origin, axis sign, limit, and active visual transform is read directly from the pinned URDF. URDF fixed-axis RPY is converted as `Rz(yaw) Ry(pitch) Rx(roll)` to MJCF quaternion `w x y z`; there is no mesh rescaling or hand-tuned kinematic approximation.

| q index | Source/MJCF joint | Lower | Upper | Home |
|---:|---|---:|---:|---:|
| 0 | `j0` | -6.2832 | 6.2832 | 0 |
| 1 | `j1` | -0.0005 | 3.15 | 1.0 |
| 2 | `j2` | -0.1 | 3.14 | 1.4 |
| 3 | `j3` | -6.2832 | 6.2832 | 0 |
| 4 | `j4` | -3.14 | 3.14 | 0.7 |
| 5 | `end_differential_controller/j5` | -1.57 | 1.57 | 0 |
| 6 | `end_differential_controller/j6` | -6.2831853 | 6.2831853 | 0 |

The `home` keyframe stores this posture. The `arm_tool` site is exactly the source `ee_link` frame. Its fixed transform relative to `j6_link` is translation `[0, 0.06161, -0.15266]` m and RPY `[1.570796326794895, 0, 0]`. Home tool position is `[0.15134046497099157, 0.06274965629505258, 0.8879899548931535]` m. The home full pose Jacobian has rank 6; its smallest singular value is approximately `0.07766016` with the unweighted metres/radians convention. This indicates a useful starting posture, not a physical accuracy specification.

The source URDF sets velocity and effort limits to zero placeholders. The MJCF joint `user[0]` stores **1 rad/s as a mock demonstration speed cap**, not a hardware specification. Both end differential joints remain independent generalized coordinates; physical coupled motor transmission is outside this kinematic model.

## Deliberate simulation boundary

The scene supports a mock controller that writes joint position/velocity and calls `mj_forward`. It is **not a torque/dynamics model**: gravity is zero, there are no actuators, and each of the seven moving links has explicit placeholder mass `1 kg` and isotropic inertia `0.001 kg m²`. Do not use `mj_step` results to infer physical arm behavior.

The source inertias are archived unchanged and audited in `models/rm26_arm/verification.json`. In particular, `j1_link` has principal moments `[0.0018755555, 0.1038748922, 0.1204991423] kg m²`; the largest exceeds the sum of the other two by `0.0147486946 kg m²`, so it is not a physically admissible inertia tensor. No silent balancing or physical interpretation is applied.

All meshes, the plane, and the mount are **visual only** (`contype=0`, `conaffinity=0`). MuJoCo convex hull contacts would not reproduce the detailed non-convex CAD. There is no self-collision, floor collision, collision avoidance, gripper contact, torque feedback, or workspace feasibility claim. Joint-limited configurations can intersect the ground or other links. The original meshes are preserved byte-for-byte and show geometry only; precise kinematics here means agreement with the source URDF, not calibration against the real robot.

## Reproduce and verify

From the repository root:

```bash
# Download only missing pinned model files, then regenerate the MJCF.
python scripts/import_rm26_arm.py --download

# Offline regeneration and verification against the installed C++ MuJoCo SDK.
python scripts/import_rm26_arm.py --verify
# Override the SDK location if needed:
python scripts/import_rm26_arm.py --verify --mujoco-dir /path/to/mujoco-3.3.7
```

Generation uses Python standard libraries. Verification additionally uses NumPy and a C++17 compiler. Downloading requires about 12.65 MB and uses atomic `.part` replacement with cleanup. Existing files are checked against the saved manifest before use. The verifier compiles a temporary C++ program, loads the actual MJCF through MuJoCo, calls `mj_forward`, and compares all 14 retained body frames plus the tool site against an independent URDF matrix-chain implementation. Its temporary directory is automatically removed.

The check includes home, zero, simultaneous lower and upper limits, and 64 reproducible random configurations (`seed=26`): **68 configurations total**. It verifies seven generalized positions/velocities, zero actuators, disabled contacts, and every archived source checksum. On MuJoCo 3.3.7 the maximum absolute position error was `4.44e-16 m`, and the maximum rotation-matrix element error was `1.33e-15`. `verification.json` records results, source inertia audit, home Jacobian singular values, FK fixtures, and generated scene SHA-256. Verification needs no renderer or GPU.

## Source and license provenance

`reference/rm26_arm/source.urdf` and `package.xml` are exact upstream bytes. `reference/rm26_arm/manifest.json` records repository, commit, upstream paths, local paths, byte counts, and SHA-256 hashes for both files and the ten required STL assets. No whole repository or chassis meshes were downloaded.

The upstream package's `package.xml` declares `BSD`. No standalone license text was found in the upstream repository root or the model package directory at the pinned commit. The exact declaration and upstream attribution are preserved; this importer does not assert a particular BSD variant or invent copyright terms. Imported asset licensing remains governed by the upstream authors' terms.
