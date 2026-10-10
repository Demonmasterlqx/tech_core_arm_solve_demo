# Planning benchmark

Planning time includes the planner's validation. Cold model load, atlas fitting, and denser recheck using the same validator are separate. The recheck doubles sampling density; it is not an independent validation algorithm.
The positive aggregate excludes negative cases. Negative-case rejection is expected; the all aggregate mixes both and must not be used to claim positive-case acceleration.
All-run timings include failed/infeasible runs. Paired success-only speedup compares the same case and repeat; values above 1 favor the accelerated method.

| Split | Method | Success | All p50 / p95 / max (s) | Success p50 / p95 / max (s) |
|---|---|---:|---:|---:|
| positive | adaptive_lm | 30/60 | 0.422228 / 0.762296 / 0.792272 | 0.637551 / 0.791351 / 0.792272 |
| positive | atlas | 30/60 | 0.423122 / 0.759779 / 0.803601 | 0.635031 / 0.798507 / 0.803601 |
| positive | baseline | 30/60 | 0.433774 / 0.773007 / 0.806777 | 0.650992 / 0.804760 / 0.806777 |
| positive | bounded_qp | 30/60 | 0.421964 / 0.759435 / 0.794757 | 0.638956 / 0.791754 / 0.794757 |
| positive | full_step | 30/60 | 0.422317 / 0.756922 / 0.794564 | 0.637680 / 0.791663 / 0.794564 |
| positive | predictor | 30/60 | 0.422195 / 0.757158 / 0.797598 | 0.637522 / 0.791967 / 0.797598 |
| positive | reduced_5d | 30/60 | 0.422087 / 0.756671 / 0.794722 | 0.637521 / 0.793094 / 0.794722 |
| positive | rigid_cache | 30/60 | 0.227149 / 0.356667 / 0.379631 | 0.308516 / 0.378431 / 0.379631 |
| positive | sparse_adaptive | 30/60 | 0.438400 / 1.308970 / 1.404483 | 1.138311 / 1.400605 / 1.404483 |
| development | adaptive_lm | 12/18 | 0.521790 / 0.714941 / 0.716544 | 0.560367 / 0.715507 / 0.716544 |
| development | atlas | 12/18 | 0.524255 / 0.714682 / 0.716385 | 0.558998 / 0.715283 / 0.716385 |
| development | baseline | 12/18 | 0.531237 / 0.727560 / 0.729134 | 0.570825 / 0.728116 / 0.729134 |
| development | bounded_qp | 12/18 | 0.521014 / 0.714331 / 0.716987 | 0.560057 / 0.715269 / 0.716987 |
| development | full_step | 12/18 | 0.521355 / 0.713306 / 0.714242 | 0.560225 / 0.713636 / 0.714242 |
| development | predictor | 12/18 | 0.521498 / 0.715980 / 0.717868 | 0.560022 / 0.716646 / 0.717868 |
| development | reduced_5d | 12/18 | 0.520333 / 0.714618 / 0.714671 | 0.559430 / 0.714637 / 0.714671 |
| development | rigid_cache | 12/18 | 0.240674 / 0.344035 / 0.344765 | 0.249597 / 0.344293 / 0.344765 |
| development | sparse_adaptive | 12/18 | 0.992461 / 1.300161 / 1.304149 | 1.138311 / 1.301568 / 1.304149 |
| confirmation | adaptive_lm | 18/42 | 0.191863 / 0.789702 / 0.792272 | 0.688639 / 0.791576 / 0.792272 |
| confirmation | atlas | 18/42 | 0.192995 / 0.796135 / 0.803601 | 0.701008 / 0.799516 / 0.803601 |
| confirmation | baseline | 18/42 | 0.208556 / 0.801291 / 0.806777 | 0.701665 / 0.806382 / 0.806777 |
| confirmation | bounded_qp | 18/42 | 0.191777 / 0.790029 / 0.794757 | 0.689168 / 0.792222 / 0.794757 |
| confirmation | full_step | 18/42 | 0.192097 / 0.789441 / 0.794564 | 0.689096 / 0.792387 / 0.794564 |
| confirmation | predictor | 18/42 | 0.192058 / 0.790070 / 0.797598 | 0.689016 / 0.792860 / 0.797598 |
| confirmation | reduced_5d | 18/42 | 0.192683 / 0.790070 / 0.794722 | 0.688120 / 0.794151 / 0.794722 |
| confirmation | rigid_cache | 18/42 | 0.123051 / 0.377211 / 0.379631 | 0.323816 / 0.378666 / 0.379631 |
| confirmation | sparse_adaptive | 18/42 | 0.018273 / 1.394255 / 1.404483 | 1.198493 / 1.401213 / 1.404483 |
| negative | adaptive_lm | 0/9 | 0.000979 / 0.002231 / 0.002242 | — / — / — |
| negative | atlas | 0/9 | 0.000983 / 0.008683 / 0.008698 | — / — / — |
| negative | baseline | 0/9 | 0.000978 / 0.008022 / 0.008038 | — / — / — |
| negative | bounded_qp | 0/9 | 0.000979 / 0.008493 / 0.008502 | — / — / — |
| negative | full_step | 0/9 | 0.000976 / 0.002300 / 0.002314 | — / — / — |
| negative | predictor | 0/9 | 0.000985 / 0.009234 / 0.009247 | — / — / — |
| negative | reduced_5d | 0/9 | 0.000981 / 0.008669 / 0.008700 | — / — / — |
| negative | rigid_cache | 0/9 | 0.000870 / 0.009130 / 0.009146 | — / — / — |
| negative | sparse_adaptive | 0/9 | 0.000979 / 0.008221 / 0.008224 | — / — / — |
| all | adaptive_lm | 30/69 | 0.039915 / 0.758278 / 0.792272 | 0.637551 / 0.791351 / 0.792272 |
| all | atlas | 30/69 | 0.039750 / 0.756766 / 0.803601 | 0.635031 / 0.798507 / 0.803601 |
| all | baseline | 30/69 | 0.057098 / 0.770052 / 0.806777 | 0.650992 / 0.804760 / 0.806777 |
| all | bounded_qp | 30/69 | 0.040105 / 0.755145 / 0.794757 | 0.638956 / 0.791754 / 0.794757 |
| all | full_step | 30/69 | 0.039982 / 0.754619 / 0.794564 | 0.637680 / 0.791663 / 0.794564 |
| all | predictor | 30/69 | 0.039869 / 0.754527 / 0.797598 | 0.637522 / 0.791967 / 0.797598 |
| all | reduced_5d | 30/69 | 0.039923 / 0.754527 / 0.794722 | 0.637521 / 0.793094 / 0.794722 |
| all | rigid_cache | 30/69 | 0.028036 / 0.355163 / 0.379631 | 0.308516 / 0.378431 / 0.379631 |
| all | sparse_adaptive | 30/69 | 0.008224 / 1.302272 / 1.404483 | 1.138311 / 1.400605 / 1.404483 |

| Split | Method | Paired successful n | Speedup p50 / p95 / max | Success regressions |
|---|---|---:|---:|---:|
| positive | adaptive_lm | 30 | 1.018375 / 1.022462 / 1.022943 | 0 |
| development | adaptive_lm | 12 | 1.018069 / 1.021838 / 1.022702 | 0 |
| confirmation | adaptive_lm | 18 | 1.018375 / 1.022285 / 1.022943 | 0 |
| negative | adaptive_lm | 0 | — / — / — | 0 |
| all | adaptive_lm | 30 | 1.018375 / 1.022462 / 1.022943 | 0 |
| positive | atlas | 30 | 1.016520 / 1.071314 / 1.072051 | 0 |
| development | atlas | 12 | 1.018865 / 1.023965 / 1.024291 | 0 |
| confirmation | atlas | 18 | 1.012473 / 1.071505 / 1.072051 | 0 |
| negative | atlas | 0 | — / — / — | 0 |
| all | atlas | 30 | 1.016520 / 1.071314 / 1.072051 | 0 |
| positive | bounded_qp | 30 | 1.017907 / 1.022157 / 1.023196 | 0 |
| development | bounded_qp | 12 | 1.017492 / 1.022041 / 1.022183 | 0 |
| confirmation | bounded_qp | 18 | 1.018092 / 1.022286 / 1.023196 | 0 |
| negative | bounded_qp | 0 | — / — / — | 0 |
| all | bounded_qp | 30 | 1.017907 / 1.022157 / 1.023196 | 0 |
| positive | full_step | 30 | 1.018366 / 1.022839 / 1.023294 | 0 |
| development | full_step | 12 | 1.018858 / 1.022599 / 1.023154 | 0 |
| confirmation | full_step | 18 | 1.018209 / 1.022580 / 1.023294 | 0 |
| negative | full_step | 0 | — / — / — | 0 |
| all | full_step | 30 | 1.018366 / 1.022839 / 1.023294 | 0 |
| positive | predictor | 30 | 1.018827 / 1.022896 / 1.023624 | 0 |
| development | predictor | 12 | 1.019362 / 1.022168 / 1.022235 | 0 |
| confirmation | predictor | 18 | 1.018541 / 1.023464 / 1.023624 | 0 |
| negative | predictor | 0 | — / — / — | 0 |
| all | predictor | 30 | 1.018827 / 1.022896 / 1.023624 | 0 |
| positive | reduced_5d | 30 | 1.018501 / 1.022860 / 1.023806 | 0 |
| development | reduced_5d | 12 | 1.018798 / 1.022843 / 1.023806 | 0 |
| confirmation | reduced_5d | 18 | 1.018308 / 1.022737 / 1.022997 | 0 |
| negative | reduced_5d | 0 | — / — / — | 0 |
| all | reduced_5d | 30 | 1.018501 / 1.022860 / 1.023806 | 0 |
| positive | rigid_cache | 30 | 2.137868 / 2.345670 / 2.349589 | 0 |
| development | rigid_cache | 12 | 2.197387 / 2.335402 / 2.343299 | 0 |
| confirmation | rigid_cache | 18 | 2.102391 / 2.347866 / 2.349589 | 0 |
| negative | rigid_cache | 0 | — / — / — | 0 |
| all | rigid_cache | 30 | 2.137868 / 2.345670 / 2.349589 | 0 |
| positive | sparse_adaptive | 30 | 0.574077 / 0.606695 / 0.608169 | 0 |
| development | sparse_adaptive | 12 | 0.537971 / 0.597697 / 0.598840 | 0 |
| confirmation | sparse_adaptive | 18 | 0.577806 / 0.607698 / 0.608169 | 0 |
| negative | sparse_adaptive | 0 | — / — / — | 0 |
| all | sparse_adaptive | 30 | 0.574077 / 0.606695 / 0.608169 | 0 |

## Failed runs

- development / dev_yaw / reduced_5d / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / adaptive_lm / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / predictor / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / sparse_adaptive / repeat 0: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- development / dev_yaw / atlas / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / bounded_qp / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / rigid_cache / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / baseline / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000959
- development / dev_yaw / full_step / repeat 0: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_tilt / predictor / repeat 0: ik_flip_230_iteration limit
- development / dev_tilt / sparse_adaptive / repeat 0: ik_flip_44_iteration limit
- development / dev_tilt / atlas / repeat 0: ik_flip_230_iteration limit
- development / dev_tilt / bounded_qp / repeat 0: ik_flip_230_iteration limit
- development / dev_tilt / rigid_cache / repeat 0: ik_flip_230_iteration limit
- development / dev_tilt / baseline / repeat 0: ik_flip_230_iteration limit
- development / dev_tilt / full_step / repeat 0: ik_flip_230_no descending feasible step
- development / dev_tilt / reduced_5d / repeat 0: ik_flip_230_iteration limit
- development / dev_tilt / adaptive_lm / repeat 0: ik_flip_230_no descending feasible step
- confirmation / confirm_qpos / atlas / repeat 0: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / bounded_qp / repeat 0: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / rigid_cache / repeat 0: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / baseline / repeat 0: collision_approach_13_j2_link_visual/collision_arm_j4_link_part_5_d=0.000996
- confirmation / confirm_qpos / full_step / repeat 0: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / reduced_5d / repeat 0: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / adaptive_lm / repeat 0: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / predictor / repeat 0: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / sparse_adaptive / repeat 0: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_high / rigid_cache / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / baseline / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.001000
- confirmation / confirm_high / full_step / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / reduced_5d / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / adaptive_lm / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / predictor / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / sparse_adaptive / repeat 0: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_high / atlas / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / bounded_qp / repeat 0: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_left / baseline / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000984
- confirmation / confirm_left / full_step / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / reduced_5d / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / adaptive_lm / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / predictor / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / sparse_adaptive / repeat 0: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_left / atlas / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / bounded_qp / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / rigid_cache / repeat 0: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_tilt / reduced_5d / repeat 0: ik_flip_230_iteration limit
- confirmation / confirm_tilt / adaptive_lm / repeat 0: ik_flip_230_no descending feasible step
- confirmation / confirm_tilt / predictor / repeat 0: ik_flip_230_iteration limit
- confirmation / confirm_tilt / sparse_adaptive / repeat 0: ik_flip_44_iteration limit
- confirmation / confirm_tilt / atlas / repeat 0: ik_flip_230_iteration limit
- confirmation / confirm_tilt / bounded_qp / repeat 0: ik_flip_230_iteration limit
- confirmation / confirm_tilt / rigid_cache / repeat 0: ik_flip_230_iteration limit
- confirmation / confirm_tilt / baseline / repeat 0: ik_flip_230_iteration limit
- confirmation / confirm_tilt / full_step / repeat 0: ik_flip_230_no descending feasible step
- confirmation / confirm_top / sparse_adaptive / repeat 0: ik_flip_56_iteration limit
- confirmation / confirm_top / atlas / repeat 0: ik_flip_304_iteration limit
- confirmation / confirm_top / bounded_qp / repeat 0: ik_flip_304_iteration limit
- confirmation / confirm_top / rigid_cache / repeat 0: ik_flip_304_iteration limit
- confirmation / confirm_top / baseline / repeat 0: ik_flip_304_iteration limit
- confirmation / confirm_top / full_step / repeat 0: ik_flip_304_no descending feasible step
- confirmation / confirm_top / reduced_5d / repeat 0: ik_flip_304_iteration limit
- confirmation / confirm_top / adaptive_lm / repeat 0: ik_flip_304_no descending feasible step
- confirmation / confirm_top / predictor / repeat 0: ik_flip_304_iteration limit
- confirmation / confirm_yaw_pos / atlas / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / bounded_qp / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / rigid_cache / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / baseline / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000961
- confirmation / confirm_yaw_pos / full_step / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / reduced_5d / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / adaptive_lm / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / predictor / repeat 0: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / sparse_adaptive / repeat 0: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_yaw_neg / bounded_qp / repeat 0: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / rigid_cache / repeat 0: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / baseline / repeat 0: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000725
- confirmation / confirm_yaw_neg / full_step / repeat 0: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / reduced_5d / repeat 0: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / adaptive_lm / repeat 0: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000897
- confirmation / confirm_yaw_neg / predictor / repeat 0: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / sparse_adaptive / repeat 0: collision_coarse_insert_j5_link_visual/collision_core_shell
- confirmation / confirm_yaw_neg / atlas / repeat 0: collision_lift_121_j5_link_visual/collision_core_shell_d=0.001000
- confirmation / confirm_ymax / full_step / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / reduced_5d / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / adaptive_lm / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / predictor / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / sparse_adaptive / repeat 0: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_ymax / atlas / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / bounded_qp / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / rigid_cache / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / baseline / repeat 0: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000998
- negative / reject_unreachable / reduced_5d / repeat 0: ik_approach_168_iteration limit
- negative / reject_unreachable / adaptive_lm / repeat 0: ik_approach_168_no descending feasible step
- negative / reject_unreachable / predictor / repeat 0: ik_approach_168_iteration limit
- negative / reject_unreachable / sparse_adaptive / repeat 0: ik_approach_29_iteration limit
- negative / reject_unreachable / atlas / repeat 0: ik_approach_168_iteration limit
- negative / reject_unreachable / bounded_qp / repeat 0: ik_approach_168_iteration limit
- negative / reject_unreachable / rigid_cache / repeat 0: ik_approach_168_iteration limit
- negative / reject_unreachable / baseline / repeat 0: ik_approach_168_iteration limit
- negative / reject_unreachable / full_step / repeat 0: ik_approach_168_no descending feasible step
- negative / reject_blocked / adaptive_lm / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / predictor / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / sparse_adaptive / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / atlas / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / bounded_qp / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / rigid_cache / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / baseline / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / full_step / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / reduced_5d / repeat 0: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_front_plane / predictor / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / sparse_adaptive / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / atlas / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / bounded_qp / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / rigid_cache / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / baseline / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / full_step / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / reduced_5d / repeat 0: invalid_station_front_plane
- negative / reject_front_plane / adaptive_lm / repeat 0: invalid_station_front_plane
- development / dev_yaw / adaptive_lm / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / predictor / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / sparse_adaptive / repeat 1: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- development / dev_yaw / atlas / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / bounded_qp / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / rigid_cache / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / baseline / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000959
- development / dev_yaw / full_step / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / reduced_5d / repeat 1: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_tilt / sparse_adaptive / repeat 1: ik_flip_44_iteration limit
- development / dev_tilt / atlas / repeat 1: ik_flip_230_iteration limit
- development / dev_tilt / bounded_qp / repeat 1: ik_flip_230_iteration limit
- development / dev_tilt / rigid_cache / repeat 1: ik_flip_230_iteration limit
- development / dev_tilt / baseline / repeat 1: ik_flip_230_iteration limit
- development / dev_tilt / full_step / repeat 1: ik_flip_230_no descending feasible step
- development / dev_tilt / reduced_5d / repeat 1: ik_flip_230_iteration limit
- development / dev_tilt / adaptive_lm / repeat 1: ik_flip_230_no descending feasible step
- development / dev_tilt / predictor / repeat 1: ik_flip_230_iteration limit
- confirmation / confirm_qpos / bounded_qp / repeat 1: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / rigid_cache / repeat 1: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / baseline / repeat 1: collision_approach_13_j2_link_visual/collision_arm_j4_link_part_5_d=0.000996
- confirmation / confirm_qpos / full_step / repeat 1: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / reduced_5d / repeat 1: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / adaptive_lm / repeat 1: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / predictor / repeat 1: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / sparse_adaptive / repeat 1: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_qpos / atlas / repeat 1: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_high / baseline / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.001000
- confirmation / confirm_high / full_step / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / reduced_5d / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / adaptive_lm / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / predictor / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / sparse_adaptive / repeat 1: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_high / atlas / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / bounded_qp / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / rigid_cache / repeat 1: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_left / full_step / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / reduced_5d / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / adaptive_lm / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / predictor / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / sparse_adaptive / repeat 1: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_left / atlas / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / bounded_qp / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / rigid_cache / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / baseline / repeat 1: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000984
- confirmation / confirm_tilt / adaptive_lm / repeat 1: ik_flip_230_no descending feasible step
- confirmation / confirm_tilt / predictor / repeat 1: ik_flip_230_iteration limit
- confirmation / confirm_tilt / sparse_adaptive / repeat 1: ik_flip_44_iteration limit
- confirmation / confirm_tilt / atlas / repeat 1: ik_flip_230_iteration limit
- confirmation / confirm_tilt / bounded_qp / repeat 1: ik_flip_230_iteration limit
- confirmation / confirm_tilt / rigid_cache / repeat 1: ik_flip_230_iteration limit
- confirmation / confirm_tilt / baseline / repeat 1: ik_flip_230_iteration limit
- confirmation / confirm_tilt / full_step / repeat 1: ik_flip_230_no descending feasible step
- confirmation / confirm_tilt / reduced_5d / repeat 1: ik_flip_230_iteration limit
- confirmation / confirm_top / atlas / repeat 1: ik_flip_304_iteration limit
- confirmation / confirm_top / bounded_qp / repeat 1: ik_flip_304_iteration limit
- confirmation / confirm_top / rigid_cache / repeat 1: ik_flip_304_iteration limit
- confirmation / confirm_top / baseline / repeat 1: ik_flip_304_iteration limit
- confirmation / confirm_top / full_step / repeat 1: ik_flip_304_no descending feasible step
- confirmation / confirm_top / reduced_5d / repeat 1: ik_flip_304_iteration limit
- confirmation / confirm_top / adaptive_lm / repeat 1: ik_flip_304_no descending feasible step
- confirmation / confirm_top / predictor / repeat 1: ik_flip_304_iteration limit
- confirmation / confirm_top / sparse_adaptive / repeat 1: ik_flip_56_iteration limit
- confirmation / confirm_yaw_pos / bounded_qp / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / rigid_cache / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / baseline / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000961
- confirmation / confirm_yaw_pos / full_step / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / reduced_5d / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / adaptive_lm / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / predictor / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / sparse_adaptive / repeat 1: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_yaw_pos / atlas / repeat 1: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_neg / rigid_cache / repeat 1: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / baseline / repeat 1: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000725
- confirmation / confirm_yaw_neg / full_step / repeat 1: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / reduced_5d / repeat 1: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / adaptive_lm / repeat 1: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000897
- confirmation / confirm_yaw_neg / predictor / repeat 1: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / sparse_adaptive / repeat 1: collision_coarse_insert_j5_link_visual/collision_core_shell
- confirmation / confirm_yaw_neg / atlas / repeat 1: collision_lift_121_j5_link_visual/collision_core_shell_d=0.001000
- confirmation / confirm_yaw_neg / bounded_qp / repeat 1: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_ymax / reduced_5d / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / adaptive_lm / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / predictor / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / sparse_adaptive / repeat 1: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_ymax / atlas / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / bounded_qp / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / rigid_cache / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / baseline / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000998
- confirmation / confirm_ymax / full_step / repeat 1: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- negative / reject_unreachable / adaptive_lm / repeat 1: ik_approach_168_no descending feasible step
- negative / reject_unreachable / predictor / repeat 1: ik_approach_168_iteration limit
- negative / reject_unreachable / sparse_adaptive / repeat 1: ik_approach_29_iteration limit
- negative / reject_unreachable / atlas / repeat 1: ik_approach_168_iteration limit
- negative / reject_unreachable / bounded_qp / repeat 1: ik_approach_168_iteration limit
- negative / reject_unreachable / rigid_cache / repeat 1: ik_approach_168_iteration limit
- negative / reject_unreachable / baseline / repeat 1: ik_approach_168_iteration limit
- negative / reject_unreachable / full_step / repeat 1: ik_approach_168_no descending feasible step
- negative / reject_unreachable / reduced_5d / repeat 1: ik_approach_168_iteration limit
- negative / reject_blocked / predictor / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / sparse_adaptive / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / atlas / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / bounded_qp / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / rigid_cache / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / baseline / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / full_step / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / reduced_5d / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / adaptive_lm / repeat 1: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_front_plane / sparse_adaptive / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / atlas / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / bounded_qp / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / rigid_cache / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / baseline / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / full_step / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / reduced_5d / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / adaptive_lm / repeat 1: invalid_station_front_plane
- negative / reject_front_plane / predictor / repeat 1: invalid_station_front_plane
- development / dev_yaw / predictor / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / sparse_adaptive / repeat 2: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- development / dev_yaw / atlas / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / bounded_qp / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / rigid_cache / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / baseline / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000959
- development / dev_yaw / full_step / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / reduced_5d / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_yaw / adaptive_lm / repeat 2: collision_approach_6_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- development / dev_tilt / atlas / repeat 2: ik_flip_230_iteration limit
- development / dev_tilt / bounded_qp / repeat 2: ik_flip_230_iteration limit
- development / dev_tilt / rigid_cache / repeat 2: ik_flip_230_iteration limit
- development / dev_tilt / baseline / repeat 2: ik_flip_230_iteration limit
- development / dev_tilt / full_step / repeat 2: ik_flip_230_no descending feasible step
- development / dev_tilt / reduced_5d / repeat 2: ik_flip_230_iteration limit
- development / dev_tilt / adaptive_lm / repeat 2: ik_flip_230_no descending feasible step
- development / dev_tilt / predictor / repeat 2: ik_flip_230_iteration limit
- development / dev_tilt / sparse_adaptive / repeat 2: ik_flip_44_iteration limit
- confirmation / confirm_qpos / rigid_cache / repeat 2: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / baseline / repeat 2: collision_approach_13_j2_link_visual/collision_arm_j4_link_part_5_d=0.000996
- confirmation / confirm_qpos / full_step / repeat 2: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / reduced_5d / repeat 2: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / adaptive_lm / repeat 2: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / predictor / repeat 2: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / sparse_adaptive / repeat 2: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_qpos / atlas / repeat 2: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_qpos / bounded_qp / repeat 2: collision_approach_12_j2_link_visual/collision_arm_j4_link_part_5_d=0.000999
- confirmation / confirm_high / full_step / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / reduced_5d / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / adaptive_lm / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / predictor / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / sparse_adaptive / repeat 2: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_high / atlas / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / bounded_qp / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / rigid_cache / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.000994
- confirmation / confirm_high / baseline / repeat 2: collision_approach_15_j2_link_visual/collision_arm_j4_link_part_5_d=0.001000
- confirmation / confirm_left / reduced_5d / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / adaptive_lm / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / predictor / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / sparse_adaptive / repeat 2: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_left / atlas / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / bounded_qp / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / rigid_cache / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_left / baseline / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000984
- confirmation / confirm_left / full_step / repeat 2: collision_approach_8_j2_link_visual/collision_arm_j4_link_part_5_d=0.000978
- confirmation / confirm_tilt / predictor / repeat 2: ik_flip_230_iteration limit
- confirmation / confirm_tilt / sparse_adaptive / repeat 2: ik_flip_44_iteration limit
- confirmation / confirm_tilt / atlas / repeat 2: ik_flip_230_iteration limit
- confirmation / confirm_tilt / bounded_qp / repeat 2: ik_flip_230_iteration limit
- confirmation / confirm_tilt / rigid_cache / repeat 2: ik_flip_230_iteration limit
- confirmation / confirm_tilt / baseline / repeat 2: ik_flip_230_iteration limit
- confirmation / confirm_tilt / full_step / repeat 2: ik_flip_230_no descending feasible step
- confirmation / confirm_tilt / reduced_5d / repeat 2: ik_flip_230_iteration limit
- confirmation / confirm_tilt / adaptive_lm / repeat 2: ik_flip_230_no descending feasible step
- confirmation / confirm_top / bounded_qp / repeat 2: ik_flip_304_iteration limit
- confirmation / confirm_top / rigid_cache / repeat 2: ik_flip_304_iteration limit
- confirmation / confirm_top / baseline / repeat 2: ik_flip_304_iteration limit
- confirmation / confirm_top / full_step / repeat 2: ik_flip_304_no descending feasible step
- confirmation / confirm_top / reduced_5d / repeat 2: ik_flip_304_iteration limit
- confirmation / confirm_top / adaptive_lm / repeat 2: ik_flip_304_no descending feasible step
- confirmation / confirm_top / predictor / repeat 2: ik_flip_304_iteration limit
- confirmation / confirm_top / sparse_adaptive / repeat 2: ik_flip_56_iteration limit
- confirmation / confirm_top / atlas / repeat 2: ik_flip_304_iteration limit
- confirmation / confirm_yaw_pos / rigid_cache / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / baseline / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000961
- confirmation / confirm_yaw_pos / full_step / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / reduced_5d / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / adaptive_lm / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / predictor / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / sparse_adaptive / repeat 2: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_yaw_pos / atlas / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_pos / bounded_qp / repeat 2: collision_approach_4_j2_link_visual/collision_arm_j4_link_part_5_d=0.000954
- confirmation / confirm_yaw_neg / baseline / repeat 2: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000725
- confirmation / confirm_yaw_neg / full_step / repeat 2: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / reduced_5d / repeat 2: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / adaptive_lm / repeat 2: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000897
- confirmation / confirm_yaw_neg / predictor / repeat 2: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / sparse_adaptive / repeat 2: collision_coarse_insert_j5_link_visual/collision_core_shell
- confirmation / confirm_yaw_neg / atlas / repeat 2: collision_lift_121_j5_link_visual/collision_core_shell_d=0.001000
- confirmation / confirm_yaw_neg / bounded_qp / repeat 2: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_yaw_neg / rigid_cache / repeat 2: collision_lift_116_j5_link_visual/collision_core_shell_d=0.000892
- confirmation / confirm_ymax / adaptive_lm / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / predictor / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / sparse_adaptive / repeat 2: collision_coarse_approach_j2_link_visual/collision_arm_j4_link_part_5
- confirmation / confirm_ymax / atlas / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / bounded_qp / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / rigid_cache / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / baseline / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000998
- confirmation / confirm_ymax / full_step / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- confirmation / confirm_ymax / reduced_5d / repeat 2: collision_approach_11_j2_link_visual/collision_arm_j4_link_part_5_d=0.000992
- negative / reject_unreachable / predictor / repeat 2: ik_approach_168_iteration limit
- negative / reject_unreachable / sparse_adaptive / repeat 2: ik_approach_29_iteration limit
- negative / reject_unreachable / atlas / repeat 2: ik_approach_168_iteration limit
- negative / reject_unreachable / bounded_qp / repeat 2: ik_approach_168_iteration limit
- negative / reject_unreachable / rigid_cache / repeat 2: ik_approach_168_iteration limit
- negative / reject_unreachable / baseline / repeat 2: ik_approach_168_iteration limit
- negative / reject_unreachable / full_step / repeat 2: ik_approach_168_no descending feasible step
- negative / reject_unreachable / reduced_5d / repeat 2: ik_approach_168_iteration limit
- negative / reject_unreachable / adaptive_lm / repeat 2: ik_approach_168_no descending feasible step
- negative / reject_blocked / sparse_adaptive / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / atlas / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / bounded_qp / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / rigid_cache / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / baseline / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / full_step / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / reduced_5d / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / adaptive_lm / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_blocked / predictor / repeat 2: start_collision_carried_unit_flange_0/collision_block_fixture_d=-0.020000
- negative / reject_front_plane / atlas / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / bounded_qp / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / rigid_cache / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / baseline / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / full_step / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / reduced_5d / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / adaptive_lm / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / predictor / repeat 2: invalid_station_front_plane
- negative / reject_front_plane / sparse_adaptive / repeat 2: invalid_station_front_plane
