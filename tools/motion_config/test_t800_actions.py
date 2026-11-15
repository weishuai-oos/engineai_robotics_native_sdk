"""Regressions for the T800 task graph and its deployed policy assets.

Uses the SDK's existing PyYAML and NumPy tooling; no robot or SDK libraries are
needed. Run with ``python3 -B -m unittest discover -s tools/motion_config -v``.
"""

from pathlib import Path
import hashlib
import json
import unittest

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = REPO_ROOT / "assets/config/t800"
REPLACED_TAGS = {"rl_dance_example", "rl_victory_example", "rl_ridicule", "rl_celebration"}
REPLACED_TAGS.update({"rl_supermimic_come_here", "rl_supermimic_left_hook",
                      "rl_supermimic_pull_hammer", "rl_supermimic_right_high_kick"})
ARCHIVED_DIRS = {
    "rl_dance_example", "rl_victory_example",
    "rl_ridicule_motion1_baseline_scratch_3090_j9829_m26500",
    "rl_prone_to_stance_hybrid_9825_m29500",
    "rl_walking_leolab_example",
    "rl_supermimic_zq_right_high_kick_20261102_j13551_m184000",
}
TAUNT_BACKUP_PACKAGES = {
    "rl_supermimic_zq_come_here_02_j13904_1_m36000",
    "rl_supermimic_zq_come_here_03_j13904_2_m38000",
}
COMBINED_WALKING_PROFILE = "rl_walking_leolab_ankle_combined_v2_140000"
IMPACT_WALKING_PROFILE = "rl_walking_leolab_impact_v2_154500"
PRONE_PACKAGE = "rl_supermimic_prone_j12768_m22000"
RECOVERY_ENTRY_KEYS = {
    "entry_transition_enabled", "entry_transition_duration",
    "entry_transition_min_duration", "entry_transition_max_duration",
    "entry_transition_max_joint_velocity", "entry_transition_max_joint_acceleration",
    "entry_transition_reference_pose_weight", "entry_transition_source_tracking_error",
    "entry_transition_to_reference_pose", "entry_transition_reference_pose_tolerance",
    "entry_transition_reference_pose_settle_cycles",
    "entry_transition_reference_pose_arm_stiffness_scale",
}
ARCHIVED_SUPER_MOTIONS = {
    "supermimic_come_here": (
        "rl_supermimic_come_here", "B", "rl_supermimic_zq_come_here_01_j13904_0_m36000"),
    "supermimic_left_hook": (
        "rl_supermimic_left_hook", "Y", "rl_supermimic_zq_left_hook_02_j13564_m82000"),
    "supermimic_pull_hammer": (
        "rl_supermimic_pull_hammer", "X", "rl_supermimic_zq_pull_hammer_20261102_j13551_m182000"),
    "supermimic_right_high_kick": (
        "rl_supermimic_right_high_kick", "A", "rl_supermimic_lab3_zq_right_high_kick_20261102_j14030_m90000"),
}
# Five numbered policies previously promoted to the competition keys.
ARCHIVED_PROMOTED_STRIKES = {
    'supermimic_left_hook_26000': ('rl_left_front_kick_002_complete_heightfall', 1, ['RB', 'CROSS_X_UP'], 'rl_supermimic_s2bf_zq_left_hook_02_j13961_3_m26000'),
    'supermimic_left_straight_12000': ('rl_roundhouse_kick', 3, ['RB', 'CROSS_X_DOWN'], 'rl_supermimic_s2air20_zq_left_straight_05_j14393_0_m12000'),
    'supermimic_left_flick_mid_8000': ('rl_straight_punch_L_improved', 4, ['RB', 'CROSS_Y_LEFT'], 'rl_supermimic_s2air20_zq_left_flick_mid_01_j14282_1_m8000'),
    'supermimic_pull_hammer_56000': ('rl_left_hook_001_improved', 2, ['CROSS_X_UP', 'CROSS_X_DOWN', 'CROSS_Y_RIGHT'], 'rl_supermimic_s2air25_zq_pull_hammer_20261102_j14016_4_m56000'),
    'supermimic_pull_hammer_22000': ('rl_punch_example', 5, ['RB', 'CROSS_Y_RIGHT'], 'rl_supermimic_s2air20_zq_pull_hammer_20261102_j14282_2_m22000'),
}
REPLACED_TAGS.update(value[0] for value in ARCHIVED_PROMOTED_STRIKES.values())
REPLACED_TAGS.update("rl_" + motion for motion in ARCHIVED_PROMOTED_STRIKES)
ARCHIVED_SUPER_PACKAGES = {
    "old/11.14/" + directory for _, _, directory in ARCHIVED_SUPER_MOTIONS.values()
} | {
    "old/11.14/numbered_actions/" + value[3]
    for value in ARCHIVED_PROMOTED_STRIKES.values()
}

# These action IDs and physical bindings are the requested deployment contract.
DEPLOYED_ACTIONS = {
    1: (["LB", "CROSS_X_UP"], "rl_supermimic_stage2_guard_09_zq_xjh_3_5_01_strike_10_i012000"),
    2: (["LB", "CROSS_X_DOWN"], "rl_supermimic_stage1_ankle_pd40_2_08_zq_xjh_3_5_01_strike_08_i010000"),
    3: (["LB", "CROSS_Y_LEFT"], "rl_supermimic_unstable_original5_plateau_08_i006000"),
    4: (["LB", "CROSS_Y_RIGHT"], "rl_supermimic_stage2_ankle_pd40_2_08_i008000"),
    5: (["RB", "CROSS_X_UP"], "rl_supermimic_stage1_ankle_pd40_2_12_zq_xjh_3_8_01_strike_04_i010000"),
    6: (["RB", "CROSS_X_DOWN"], "rl_supermimic_stage2_selected6000_12_i006000"),
    7: (["RB", "CROSS_Y_LEFT"], "rl_supermimic_stage2_ankle_pd40_2_12_i008000"),
    8: (["RB", "CROSS_Y_RIGHT"], "rl_supermimic_stage2_selected6000_06_i006000"),
    9: (["RB", "A"], "rl_supermimic_stage2_guard_11_zq_xjh_3_7_01_strike_09_i012000"),
    10: (["RB", "B"], "rl_supermimic_stage2_guard_19_zq_xjh_4_4_01_strike_01_i016000"),
    11: (["RB", "X"], "rl_supermimic_stage2_guard_20_zq_xjh_4_5_01_strike_06_i016000"),
    12: (["RB", "Y"], "rl_supermimic_unstable_original5_latest_14_i026000"),
    14: (["BACK", "START", "CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT", "CROSS_Y_RIGHT"],
         "rl_supermimic_taunt_01_zq_xjh_5_1_01_strike_01_i036000"),
    15: (["CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT"],
         "rl_supermimic_taunt_02_zq_xjh_5_1_01_strike_02_i036000"),
    16: (["CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_RIGHT"],
         "rl_supermimic_taunt_03_zq_xjh_5_1_01_strike_03_i036000"),
}
DEPLOYED_MOTIONS = [f"supermimic_action_{number:02d}" for number in DEPLOYED_ACTIONS]
NEW_TAGS = {"rl_" + motion for motion in DEPLOYED_MOTIONS}
SUPER_PACKAGES = {"numbered_actions/" + package for _, package in DEPLOYED_ACTIONS.values()} | {PRONE_PACKAGE}
REMOVED_NUMBERED_PACKAGES = {
    "rl_supermimic_local_clip_zq_xjh_3_10_02_strike_04",
    "rl_supermimic_local_clip_zq_xjh_3_5_01_strike_08",
    "rl_supermimic_local_clip_zq_xjh_3_5_01_strike_10",
    "rl_supermimic_local_clip_zq_xjh_3_7_01_strike_09",
    "rl_supermimic_local_clip_zq_xjh_3_8_01_strike_04",
    "rl_supermimic_local_clip_zq_xjh_3_9_01_strike_05",
    "rl_supermimic_local_clip_zq_xjh_4_10_01_strike_02",
    "rl_supermimic_local_clip_zq_xjh_4_14_01_strike_04",
    "rl_supermimic_local_clip_zq_xjh_4_1_01_strike_07",
    "rl_supermimic_local_clip_zq_xjh_4_2_01_strike_02",
    "rl_supermimic_local_clip_zq_xjh_4_3_02_strike_05",
    "rl_supermimic_local_clip_zq_xjh_4_4_01_strike_01",
    "rl_supermimic_local_clip_zq_xjh_4_5_01_strike_06",
}

COMPETITION_ASSET_HASHES = {
    'rl_left_front_kick_002_complete_heightfall': {'default.yaml': 'eef561e1b83f6a90657d26fbc5938d35c448e2842632e0c9faec9cee994b19b7', 'policies/policy.mnn': 'e0f2af34ff0c7290ff8dfd04243de6bfc983d4612d3237f25db7f49d8d1d57cd', 'trajectories/left_front_kick_002_terminal_hold_1s.npz': 'f7dc8d50a9c4e553b6ff4da9b8457461a8a57b1b46d1eefff2bd313c88f8a523'},
    'rl_roundhouse_kick': {'default.yaml': '10b09c98fadd9efd22771dbf01ec85b41fa70b26ccd293b507d578ae4d09210f', 'policies/policy.mnn': '0fbf95134b518cdc59f0fa0c70ba5fc74f773eae2ce5d6efd11fbbb01b6a6ca8', 'trajectories/roundhouse_kick_001_retimed_terminal_hold_1s.npz': '26826a6be34403d79eb38960a23ee4a7356501a5f714ad8c315e02291ef38b0d'},
    'rl_straight_punch_L_improved': {'default.yaml': '0e22a422fb6ed61794e70053f2fad89fdf07c4ca985c80dfcbab2b44e056dcb5', 'policies/policy.mnn': '8d7a2013384689818bd4397773f67d169623c0cba656447c007a96233d473a2b', 'trajectories/straight_punch_L_terminal_hold_0p5s.npz': 'c2304b8cbccddd796f12056f5264f97642936396d89cd98d6b0774945c3fedf0'},
    'rl_left_hook_001_improved': {'default.yaml': '6c7fb598c151d54dac903561e9edaf78095c6e84478d789fee958ce5d9c8d842', 'policies/policy.mnn': '176efc1dae758c4c4d950676d5b13678bae93cfc98e64b645cf4477bafd07b76', 'trajectories/left_hook_001_terminal_hold_0p5s.npz': '52baa6dd3d1f49a869ae5179dd03da6b001535709773133d1873ecb1b2a8889c'},
    'rl_punch_example': {'default.yaml': '25a7e7fc75274683c01ff898387c2940bdb65397d8c7ff80875272d2170e459d', 'policies/policy.mnn': '4306b1ffcc6028a3f1ac72011afc7e0a440d86347a3fa7cb6f084527d79d009b', 'trajectories/Punch_Swing_L_50hz.npz': '8d55be2b9cbb795b7fb68a29164cddebd63de7ab2c9526fd745fa7adfdf221ed'},
}

# Retained runtime bytes and parsed settings captured before asset cleanup.
# These checks cover PD/timing/order without keeping export reports or fixtures.
RUNTIME_PACKAGES = {
    "old/rl_dance_example": (
        "ef4ea6dc6d2c0796b21709eca370a8b40760f8aeea87c85ab28f4f1471fef806", {
            "policies/dance.mnn": "4a6b15e431293b3c3247b9b8667c4dc35c23cce8f656b39a13c3d3439d82adcf",
            "trajectories/dance.npz": "b09015db35377a43a087b27b0a1b6502a83a61c6b2d12317fa2aeb5551b8f1f6",
        }),
    "old/rl_prone_to_stance_hybrid_9825_m29500": (
        "028aa76866a28a48512576ede10720c5790f6c257cc5fbbb0a67ce2457219390", {
            "policies/policy.mnn": "e73a4f931dee14d4ced52a1413f868e33cb71040552ee839d09cfa1c03050b64",
            "trajectories/T800_prone_to_stance_sdk_clip_grounded_50hz.npz": "13bc791aa1d289d9fbb1ac02c017b4160800a2f20a4c79e24697e69b9b663cf8",
        }),
    "old/rl_ridicule_motion1_baseline_scratch_3090_j9829_m26500": (
        "2827e995c575b861a21184d85a70a277d39dcf9892fdc0e11951ab8cb455a907", {
            "policies/policy.mnn": "340b7b4c5b0551a64cae78227048cda5720a38cc327c6697137b6f9384539845",
            "trajectories/ridicule2_t800_startfix_50hz.npz": "a0f82df232ce6dc2f866fb3cfbf1fbf30275b49c7169b36c1ff0ebaba249a541",
        }),
    "old/rl_victory_example": (
        "4c000eb7f1a5bcddd8c3b416230a5c6f63b59a0c098d70fee88a61203c64dff0", {
            "policies/policy.mnn": "fabdca9cc838b40416a98f3f121b5d86189b8b9bd2def35ab9a7a58ae28686f0",
            "trajectories/victory_50hz.npz": "bafae21baca75d131becfd8f6d893eba745b8698222ff71382635491116a72d9",
        }),
    "old/rl_walking_leolab_example": (
        "13ece172053ba151d42067773bbd9d1409bd4f71cdceb80a6c0ec4c6cd93136d", {
            "2026-09-06-19-30-18:t800_lowcylinders_ankle_stable_1_seed42/boxing_policy.mnn": "4acb81eacd06897add5155aa3b3f4c6e31221dd5e60076890cca4a7ea4478209",
        }),
    "old/嘲讽/rl_supermimic_zq_come_here_02_j13904_1_m36000": (
        "25b6e392a859aa90abcd96bdf72e85b87af97ef5646a5eb05cfc9741ee097576", {
            "policies/policy.mnn": "849b9a646e7a0182f96e1954bdb377cf573d086fdedb016385ef21ec8d4b73ce",
            "trajectories/supermimic_reference.npz": "c37e3cc56b30584c512edf5b593d67d50b0af727b85be72e5d0df4c1c019c5d9",
        }),
    "old/嘲讽/rl_supermimic_zq_come_here_03_j13904_2_m38000": (
        "afa4de1ec1315709f39f3182b30091e57b036785d1d7f5b42a5747fb54b94431", {
            "policies/policy.mnn": "d00d13492839f0ddcb5c7703986375039c0339924a15d5dc7be655e6141c27ce",
            "trajectories/supermimic_reference.npz": "10cf49caee1b06c2b736fdbee45fc799765b5f3f2742b1f0c965afe85fcc62b5",
        }),
    "rl_supermimic_prone_j12768_m22000": (
        "3d7191c888819682a6ee81a06247923bf8f872a1cfc1c27ee5c58ad92708fb6e", {
            "policies/policy.mnn": "29d8e4c4266f0fd22b38845a884a33f934af55a743837ab4d2fa6eb0409dbf7d",
            "trajectories/supermimic_reference.npz": "06de307bf052f33dafe2d1f3429fe769218d9730fcd961ecc91679fc606df9ed",
        }),
    "old/11.14/rl_supermimic_zq_come_here_01_j13904_0_m36000": (
        "e0346e5f37a34e2ff64e69d063709a167a4087e9c37ac1774cc7987a19b4b6a9", {
            "policies/policy.mnn": "d4c79994f2bf9a2f87f5130da88647dc1c3387b2ce2baea5167839a82055c957",
            "trajectories/supermimic_reference.npz": "975b262fc6cc034fd1b039301daedd59db6265a7a360ef34bf336a80ff90ecda",
        }),
    "old/11.14/rl_supermimic_zq_left_hook_02_j13564_m82000": (
        "70844d73ee1dc4cfbb76275f5cb9d90517487dab2a0c62f78db553eb10d949ab", {
            "policies/policy.mnn": "869572355b6261064782fd2176cd6107a2a963f702c588bfe3b5159e0d5cb507",
            "trajectories/supermimic_reference.npz": "e03deeaf12837bd3ca30dd27f3b63dd81ed1e04fa390fd0802ab08dd028d5e70",
        }),
    "old/11.14/rl_supermimic_zq_pull_hammer_20261102_j13551_m182000": (
        "02faee851a2c76659fdb6956fb9f3b8d95793f2ecb11cc07e7812e0a4a62e936", {
            "policies/policy.mnn": "ef63eee96609f6080de4fa4d6509d9a1668f4144682c7b8a53ab9b4143672874",
            "trajectories/supermimic_reference.npz": "8ffda8a171e728c04de6adcf454c5afcee9a37d909a9d65cba76fd21f7fcae2d",
        }),
    "old/rl_supermimic_zq_right_high_kick_20261102_j13551_m184000": (
        "65c9d51f425a46fc149b577ef0dc9357bb6da219d1d21b8e78b55d396c089610", {
            "policies/policy.mnn": "98183bed912505954a5664d72b0af4f99d9956ef893ae20bc8b3f50f55cbb182",
            "trajectories/supermimic_reference.npz": "64598ce4d55b4e94a5554ccfda8cf3e2a64c931c80b9c1ef0fc73f6356bb9c28",
        }),
    "old/11.14/rl_supermimic_lab3_zq_right_high_kick_20261102_j14030_m90000": (
        "3e17ca18d733f232041a0bc122c33ed7ee7fed97b66fd3781b72ed68a387fded", {
            "policies/policy.mnn": "ac6ca413de2c9bb335fda54c38cf0cbe7a04947b66c7c02d1a3f7ea6b9f42d6c",
            "trajectories/supermimic_reference.npz": "64598ce4d55b4e94a5554ccfda8cf3e2a64c931c80b9c1ef0fc73f6356bb9c28",
        }),
    "rl_supine_to_stance_v1_hybrid_9843_m13000": (
        "8be78069198b27c82e4f7cb569d0cd18aa78b3a5095eb7cfa39fd48ed5c2baf2", {
            "policies/policy.mnn": "3347630790f7ff69ef0dce1581d2f252d1d6afd00837a132820c6bbce8e8db7f",
            "trajectories/T800_supine_to_stance_sdk_clip_50hz.npz": "f46be05840d992d90762c16f09f585961e55b0a328123da88380089290cf0492",
        }),
    "rl_walking_leolab_ankle_combined_v2_140000": (
        "e3d6355cfbedb4a77684f4b7ed42feddbfb29159365ef4c0d4c53ae3e0766e5c", {
            "policies/boxing_policy.mnn": "fd64bf176ca8524c3e256f877063d76751b964f79092ecc27c9e1f7dce04a235",
        }),
}


# The previous five numbered packages remain byte-for-byte recoverable.
RUNTIME_PACKAGES.update({
    'old/11.14/numbered_actions/rl_supermimic_s2bf_zq_left_hook_02_j13961_3_m26000': (
        'd711a3b9f79e5aa5e77e6ee4d9e25e826c4f1b78ca4458611a060b798f575e52', {
            'policies/policy.mnn': 'a0868cd55e9801a4e93cfe85f4aca52f6513cda14c27b32e060a7142ca850603',
            'trajectories/supermimic_reference.npz': '9daaa6ad20e910b7099fcc882de6655c0bd990c9c92d6e73b02b5136de4c386b',
        }),
    'old/11.14/numbered_actions/rl_supermimic_s2air25_zq_pull_hammer_20261102_j14016_4_m56000': (
        '8a48a5d80a8add5a89322be0c4c77a8533f00024c1b240ad4af242a9e0307008', {
            'policies/policy.mnn': 'c6f2b712f107ed19745b34439d0d84de09c30641616c16a6fd793158bf20fbd3',
            'trajectories/supermimic_reference.npz': '7bd0811c2d5fd15b4f0ee897d7bbe8f70ae3c47ae27143a58439772b60940a2a',
        }),
    'old/11.14/numbered_actions/rl_supermimic_s2air20_zq_left_straight_05_j14393_0_m12000': (
        'f2ab39ea1cdd1cbe7096afa8fc61680d3b4756fcd55453e305fea232403a7098', {
            'policies/policy.mnn': 'a4f03b6ec03e44f7da438f04a40d41b950078d38f6588a33b6979f40557db4bd',
            'trajectories/supermimic_reference.npz': '7095ddeed737db671dc8eeff63330176cb5e5ce3bdbadd03cac7ad6d863d8794',
        }),
    'old/11.14/numbered_actions/rl_supermimic_s2air20_zq_left_flick_mid_01_j14282_1_m8000': (
        '29f8d9ca71cd96ea279976a6974ba5de0067cebe63415df30f36b3ccb5dc82e7', {
            'policies/policy.mnn': 'f0ef57bde41e288c87e0efb8e0a5526f3b3be54b3ca6c309366d6613327c4aa8',
            'trajectories/supermimic_reference.npz': '00a10ee11a23cd7a047920bee1f2bd721092bf7b820f9489a7fbff291b1ba7cf',
        }),
    'old/11.14/numbered_actions/rl_supermimic_s2air20_zq_pull_hammer_20261102_j14282_2_m22000': (
        '7bfddfda9e586f97a867aa816a6c4a11b5751c5815f411a5eda22e6731549995', {
            'policies/policy.mnn': '25025717c587110326a709f6774f0e95a3694d6c7c9e4f3152f27de653a60d18',
            'trajectories/supermimic_reference.npz': '7bd0811c2d5fd15b4f0ee897d7bbe8f70ae3c47ae27143a58439772b60940a2a',
        }),
})

# These nine motions retain their keys, runners and periods after migration.
PRESERVED_MOTIONS = {
    "walk": (0.01, ["LB", "B"], "rl_walking_example_runner", "rl_walking_example"),
    "walk_impact": (0.02, ["LB", "Y"], "rl_walking_leolab_example_runner", "rl_walking_leolab_impact_v2"),
    "walk_leo": (0.02, ["LB", "A"], "rl_walking_leolab_example_runner", "rl_walking_leolab_example"),
    "leo_taunt": (0.02, ["CROSS_Y_LEFT", "CROSS_Y_RIGHT", "B"],
                  "rl_walking_leolab_example_runner", "rl_walking_leolab_taunt"),
    "walk_leo_terrain": (0.02, ["LB", "X"], "rl_walking_leolab_example_runner",
                         "rl_walking_leolab_terrain_example"),
    "supine_to_stance": (0.02, ["START", "CROSS_X_UP"], "rl_dance_example_runner", "rl_supine_to_stance"),
    "getup": (0.02, ["LB", "START"], "rl_getup_example_runner", "rl_getup_example"),
    "getup2": (0.02, ["LB", "BACK"], "rl_getup_example_runner", "rl_getup2_example"),
    "stance_to_supine": (0.01, ["START", "CROSS_X_DOWN"],
        "rl_mimic_trajectory_runner", "rl_stance_to_supine"),
}

PRESERVED_SIM_SCOPES = {
    "resident_task": "task_resident/sim",
    "motion_task": "task_motion/default",
    "global_options": "global_options/release",
    "lcm": "lcm/default",
    "google_logger": "google_logger/default",
    "motor": "motor/default",
    "model": "model/default",
    "base_state_estimator": "base_state_estimator/default",
    "motion_state_estimator": "motion_state_estimator/default",
    "parallel_ankle": "parallel_ankle/default",
    "joint_motor_transform": "joint_motor_transform_runner/default",
    "passive": "passive/default",
    "pd_stand": "pd_stand/default",
    "pd_stand_x": "pd_stand/pose_x",
    "pd_stand_y": "pd_stand/pose_y",
    "rl_walking_example": "rl_walking_example/default",
    "ros2_bridge": "ros2_bridge/default",
    "rl_supine_to_stance": "rl_supine_to_stance_v1_hybrid_9843_m13000/default",
    "rl_prone_to_stance": PRONE_PACKAGE + "/default",
    "rl_stance_to_supine": "rl_stance_to_supine/default",
    "rl_walking_leolab_impact_v2": IMPACT_WALKING_PROFILE + "/default",
    "rl_walking_leolab_example": COMBINED_WALKING_PROFILE + "/default",
    "rl_walking_leolab_taunt": "rl_walking_leolab_taunt/default",
    "rl_walking_leolab_terrain_example": "rl_walking_leolab_terrain_example/default",
    "rl_getup_example": "rl_getup_example/default",
    "rl_getup2_example": "rl_getup2_example/default",
}


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class T800ActionBaselineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mode = load_yaml(CONFIG_ROOT / "mode.yaml")
        cls.task_list = load_yaml(CONFIG_ROOT / "task_motion/default.yaml")["tasks"]
        cls.tasks = {task["motion"]: task for task in cls.task_list}

    def test_all_transition_targets_exist(self):
        self.assertEqual(len(self.tasks), len(self.task_list), "duplicate motion names")
        self.assertEqual(len(self.tasks), 30)
        for name, task in self.tasks.items():
            targets = list(task.get("manual_transition", []))
            if task.get("auto_transition"):
                targets.append(task["auto_transition"])
            for target in targets:
                with self.subTest(source=name, target=target):
                    self.assertIn(target, self.tasks)

    def test_modes_keep_required_scopes_and_cover_task_parameter_tags(self):
        self.assertEqual(set(self.mode["mode"]), {"sim", "robot"})
        for name, entries in self.mode["mode"].items():
            scopes = {entry["tag"]: entry["scope"] for entry in entries}
            self.assertEqual(len(scopes), len(entries), f"duplicate tags in {name}")
            expected = dict(PRESERVED_SIM_SCOPES)
            if name == "robot":
                expected.update(resident_task="task_resident/robot", lcm="lcm/robot", pd_stand="pd_stand/robot")
            stable = {tag: scope for tag, scope in scopes.items() if tag not in REPLACED_TAGS | NEW_TAGS}
            self.assertEqual(stable, expected)
            for tag, scope in scopes.items():
                with self.subTest(mode=name, tag=tag):
                    self.assertTrue((CONFIG_ROOT / (scope + ".yaml")).is_file(), scope)
            for task in self.task_list:
                for runner in task.get("runner", []):
                    if runner.get("enabled") and "param_tag" in runner:
                        with self.subTest(mode=name, motion=task["motion"]):
                            self.assertIn(runner["param_tag"], scopes)

    def test_nine_other_motions_keep_keys_runners_tags_and_periods(self):
        self.assertEqual(len(PRESERVED_MOTIONS), 9)
        walking = ["walk", "walk_impact", "walk_leo", "walk_leo_terrain"]
        returns = ["passive", "pd_stand"] + walking
        transitions = {
            name: (["passive", "pd_stand"] + [other for other in walking if other != name] +
                   (["leo_taunt"] if name == "walk_leo" else []) +
                   ["getup", "getup2", "stance_to_supine"] + DEPLOYED_MOTIONS, None)
            for name in walking
        }
        transitions.update({name: (returns, "walk_leo") for name in ["getup", "getup2"]})
        transitions.update({
            "leo_taunt": (["walk_leo"], "walk_leo"),
            "supine_to_stance": (["passive", "pd_stand", "walk_leo"], "walk_leo"),
            "stance_to_supine": (["passive", "pd_stand"], "passive"),
        })
        for name, expected in PRESERVED_MOTIONS.items():
            with self.subTest(motion=name):
                task = self.tasks[name]
                period, key, runner, tag = expected
                self.assertEqual(task["period"], period)
                self.assertEqual(task["key"], key)
                self.assertEqual(task["runner"], [{"name": runner, "enabled": True, "param_tag": tag}])
                self.assertEqual((task["manual_transition"], task.get("auto_transition")), transitions[name])

    def test_active_policy_and_trajectory_paths_exist(self):
        for name, entries in self.mode["mode"].items():
            for entry in entries:
                config_path = CONFIG_ROOT / (entry["scope"] + ".yaml")
                config = load_yaml(config_path)
                # SuperMimic exports keep their package-relative asset paths;
                # the older SDK runners resolve paths from the product root.
                asset_root = config_path.parent if config.get("method") == "supermimic" else CONFIG_ROOT
                for key in ("policy_file", "trajectory_file_npz", "reference_file"):
                    if key not in config:
                        continue
                    with self.subTest(mode=name, tag=entry["tag"], key=key):
                        self.assertTrue(config[key], "active asset path is empty")
                        path = asset_root / config[key]
                        self.assertTrue(path.is_file(), path)
                        self.assertGreater(path.stat().st_size, 0)

    def test_combined_walking_profile_replaces_lb_a_and_matches_terrain_contract(self):
        combined_path = CONFIG_ROOT / COMBINED_WALKING_PROFILE / "default.yaml"
        terrain_path = CONFIG_ROOT / "rl_walking_leolab_terrain_example/default.yaml"
        combined = load_yaml(combined_path)
        terrain = load_yaml(terrain_path)
        self.assertEqual(self.tasks["walk_leo"]["key"], ["LB", "A"])
        self.assertEqual(self.tasks["walk_leo"]["runner"][0]["param_tag"],
                         "rl_walking_leolab_example")
        self.assertEqual(combined["num_observations"], 84)
        self.assertEqual(combined["num_actions"], 25)
        self.assertTrue(combined["policy_recurrent"])
        self.assertEqual(combined["policy_file"],
                         COMBINED_WALKING_PROFILE + "/policies/boxing_policy.mnn")
        self.assertEqual([i for i, (a, b) in enumerate(zip(
            combined["joint_damping"], terrain["joint_damping"])) if a != b], [4, 5, 10, 11])
        for index in (4, 5, 10, 11):
            self.assertEqual(combined["joint_damping"][index], 2.0)
        for key in combined:
            if key not in {"policy_file", "joint_damping"}:
                self.assertEqual(combined[key], terrain[key], key)

    def test_taunt_policy_does_not_depend_on_archived_standard_walking_directory(self):
        taunt = load_yaml(CONFIG_ROOT / "rl_walking_leolab_taunt/default.yaml")
        terrain = load_yaml(CONFIG_ROOT / "rl_walking_leolab_terrain_example/default.yaml")
        self.assertNotIn("rl_walking_leolab_example/", taunt["policy_file"])
        taunt_policy = CONFIG_ROOT / taunt["policy_file"]
        terrain_policy = CONFIG_ROOT / terrain["policy_file"]
        self.assertTrue(taunt_policy.is_file())
        self.assertTrue(terrain_policy.is_file())
        self.assertEqual(file_sha256(taunt_policy), file_sha256(terrain_policy))

    def test_impact_walking_profile_replaces_lb_y_with_combined_control_contract(self):
        impact = load_yaml(CONFIG_ROOT / IMPACT_WALKING_PROFILE / "default.yaml")
        combined = load_yaml(CONFIG_ROOT / COMBINED_WALKING_PROFILE / "default.yaml")
        self.assertNotIn("walk_custom", self.tasks)
        task = self.tasks["walk_impact"]
        self.assertEqual(task["key"], ["LB", "Y"])
        self.assertEqual(task["period"], 0.02)
        self.assertEqual(task["runner"], [{"name": "rl_walking_leolab_example_runner",
                                         "enabled": True, "param_tag": "rl_walking_leolab_impact_v2"}])
        self.assertEqual(impact["policy_file"], IMPACT_WALKING_PROFILE + "/policies/boxing_policy.mnn")
        self.assertEqual(impact["num_observations"], 84)
        self.assertEqual(impact["num_actions"], 25)
        self.assertTrue(impact["policy_recurrent"])
        self.assertEqual(impact["default_joint_pos"][-2:], [0.052337, 0.621251])
        for index in (4, 5, 10, 11):
            self.assertEqual(impact["joint_stiffness"][index], 40)
            self.assertEqual(impact["joint_damping"][index], 2.0)
        for key in combined:
            if key != "policy_file":
                self.assertEqual(impact[key], combined[key], key)


class T800SuperMimicMigrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = CONFIG_ROOT / "old"
        cls.mode = load_yaml(CONFIG_ROOT / "mode.yaml")
        cls.tasks = {task["motion"]: task for task in
                     load_yaml(CONFIG_ROOT / "task_motion/default.yaml")["tasks"]}

    def test_old_actions_keep_only_runtime_files_and_remain_inactive(self):
        self.assertEqual({path.name for path in self.old.iterdir()},
                         ARCHIVED_DIRS | {"嘲讽", "上次比赛", "11.14", "README.md"})
        for directory in ARCHIVED_DIRS:
            self.assertFalse((CONFIG_ROOT / directory).exists(), directory)
            self.assertTrue((self.old / directory).is_dir(), directory)
        self.assertEqual({path.name for path in (self.old / "嘲讽").iterdir()},
                         TAUNT_BACKUP_PACKAGES | {"README.md"})
        for directory in TAUNT_BACKUP_PACKAGES:
            self.assertFalse((CONFIG_ROOT / directory).exists(), directory)
            self.assertTrue((self.old / "嘲讽" / directory).is_dir(), directory)
        for mode, entries in self.mode["mode"].items():
            for entry in entries:
                with self.subTest(mode=mode, tag=entry["tag"]):
                    self.assertNotIn(entry["tag"], REPLACED_TAGS)
                    self.assertNotIn(entry["scope"].split("/", 1)[0], ARCHIVED_DIRS | {"old"})
        self.assertTrue({"dance", "victory", "ridicule", "celebration"}.isdisjoint(self.tasks))

    def test_fifteen_actions_share_the_numbered_assets_in_both_modes(self):
        catalog = json.loads((CONFIG_ROOT / "numbered_actions/catalog.json").read_text())
        self.assertEqual(catalog["schema_version"], 1)
        self.assertEqual(catalog["capacity"], 50)
        numbered = {action["id"]: action for action in catalog["actions"]}
        self.assertEqual(len(numbered), len(catalog["actions"]), "duplicate catalog IDs")
        self.assertEqual(set(numbered), set(DEPLOYED_ACTIONS))
        self.assertNotIn(13, numbered)
        self.assertEqual({task for task in self.tasks if task.startswith("supermimic_")},
                         set(DEPLOYED_MOTIONS))
        ready = {"pd_stand", "pd_stand_x", "pd_stand_y", "walk",
                 "walk_impact", "walk_leo", "walk_leo_terrain"}
        for number, (key, package) in DEPLOYED_ACTIONS.items():
            motion = f"supermimic_action_{number:02d}"
            with self.subTest(motion=motion, number=number):
                task = self.tasks[motion]
                tag = "rl_" + motion
                self.assertEqual(task["key"], key)
                self.assertEqual(task["period"], 0.02)
                self.assertEqual(task["runner"], [{"name": "rl_supermimic_runner",
                                 "enabled": True, "param_tag": tag}])
                self.assertEqual(task["auto_transition"], "walk_leo")
                self.assertEqual(task["manual_transition"], ["passive", "pd_stand", "walk",
                                 "walk_impact", "walk_leo", "walk_leo_terrain"])
                self.assertEqual(numbered[number]["package"], package)
                for entries in self.mode["mode"].values():
                    scopes = {entry["tag"]: entry["scope"] for entry in entries}
                    self.assertEqual(scopes[tag], "numbered_actions/" + package + "/default")
                entries = {source for source, state in self.tasks.items()
                           if motion in state.get("manual_transition", [])}
                self.assertEqual(entries, ready)
                config = load_yaml(CONFIG_ROOT / "numbered_actions" / package / "default.yaml")
                self.assertEqual(config["hold_seconds"], 0.0)
                expected_skip = 0 if number in {2, 5, 14, 15, 16} else 100
                self.assertEqual(config.get("skip_terminal_hold_frames", 0), expected_skip)
                self.assertEqual(config.get("terminal_hold_frames", 0), expected_skip)
                self.assertFalse(config.get("entry_transition_to_reference_pose", False))

    def test_previous_numbered_packages_are_removed_from_active_directory(self):
        for package in REMOVED_NUMBERED_PACKAGES:
            with self.subTest(package=package):
                self.assertFalse((CONFIG_ROOT / "numbered_actions" / package).exists())
                self.assertFalse(any(package in entry["scope"]
                                     for entries in self.mode["mode"].values()
                                     for entry in entries))
        self.assertFalse((CONFIG_ROOT / "numbered_actions" /
                          "rl_supermimic_unstable_original5_latest_15_i026000").exists())

    def test_deployed_numbered_packages_only_contain_required_runtime_assets(self):
        root = CONFIG_ROOT / "numbered_actions"
        self.assertEqual({path.name for path in root.iterdir()},
                         {"catalog.json"} | {package for _, package in DEPLOYED_ACTIONS.values()})
        for number, (_, package) in DEPLOYED_ACTIONS.items():
            directory = root / package
            with self.subTest(number=number):
                self.assertEqual({path.relative_to(directory).as_posix()
                                  for path in directory.rglob("*") if path.is_file()},
                                 {"default.yaml", "policies/policy.mnn",
                                  "trajectories/supermimic_reference.npz"})

    def test_physical_key_masks_are_unique_and_unused_combinations_stay_empty(self):
        names = ("LB", "RB", "A", "B", "X", "Y", "BACK", "START",
                 "CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT", "CROSS_Y_RIGHT")
        masks = {}
        for name, task in self.tasks.items():
            with self.subTest(motion=name):
                self.assertTrue(task["key"])
                self.assertEqual(len(task["key"]), len(set(task["key"])))
                mask = sum(1 << names.index(key) for key in set(task["key"]))
                self.assertNotIn(mask, masks, f"{name} collides with {masks.get(mask)}")
                masks[mask] = name
        # RT+up uses a reserved marker; plain RT remains unassigned.
        self.assertNotIn((1 << 8) | (1 << 9), masks)
        self.assertEqual(masks[0xFC0], "supermimic_action_14")
        self.assertEqual(masks[(1 << 1) | (1 << 2)], "supermimic_action_09")
        self.assertTrue(set(masks).isdisjoint({0xF00 | number for number in range(1, 51)}))

    def test_november_14_archive_preserves_all_replaced_packages_and_catalog(self):
        folder = self.old / "11.14"
        self.assertEqual({path.name for path in folder.iterdir() if path.is_dir()},
                         {directory for _, _, directory in ARCHIVED_SUPER_MOTIONS.values()}
                         | {"numbered_actions"})
        numbered = folder / "numbered_actions"
        self.assertEqual({path.name for path in numbered.iterdir() if path.is_dir()},
                         {value[3] for value in ARCHIVED_PROMOTED_STRIKES.values()})
        self.assertEqual(file_sha256(numbered / "catalog.json"),
                         "c48894cc34d2ecc7f55effe58a80eb5a13d8549846e7c818e0fe04da95a3dd17")
        catalog = json.loads((numbered / "catalog.json").read_text())
        self.assertEqual({action["id"] for action in catalog["actions"]}, {1, 2, 3, 4, 5})
        for directory in ARCHIVED_SUPER_PACKAGES:
            package = CONFIG_ROOT / directory
            with self.subTest(package=directory):
                self.assertEqual({path.relative_to(package).as_posix()
                                  for path in package.rglob("*") if path.is_file()},
                                 {"default.yaml", "policies/policy.mnn",
                                  "trajectories/supermimic_reference.npz"})
                original = directory.removeprefix("old/11.14/")
                self.assertFalse((CONFIG_ROOT / original).exists(), original)
        self.assertTrue(set(ARCHIVED_SUPER_MOTIONS).isdisjoint(self.tasks))
        self.assertTrue(set(ARCHIVED_PROMOTED_STRIKES).isdisjoint(self.tasks))
        for entries in self.mode["mode"].values():
            self.assertTrue(REPLACED_TAGS.isdisjoint(entry["tag"] for entry in entries))
            self.assertFalse(any(entry["scope"].startswith("old/") for entry in entries))

    def test_previous_competition_packages_are_archived_without_changes(self):
        folder = CONFIG_ROOT / "old/上次比赛"
        self.assertEqual({path.name for path in folder.iterdir()}, set(COMPETITION_ASSET_HASHES))
        for directory, expected in COMPETITION_ASSET_HASHES.items():
            with self.subTest(package=directory):
                self.assertFalse((CONFIG_ROOT / directory).exists())
                package = folder / directory
                self.assertEqual({path.relative_to(package).as_posix()
                                  for path in package.rglob("*") if path.is_file()}, set(expected))
                for name, digest in expected.items():
                    self.assertEqual(file_sha256(package / name), digest, name)

    def test_runtime_packages_preserve_settings_and_asset_bytes(self):
        for directory, (expected_config, assets) in RUNTIME_PACKAGES.items():
            package = CONFIG_ROOT / directory
            with self.subTest(package=directory):
                self.assertEqual({path.relative_to(package).as_posix()
                                  for path in package.rglob("*") if path.is_file()},
                                 {"default.yaml"} | set(assets))
                config = load_yaml(package / "default.yaml")
                # Prone preparation now matches supine; the original export,
                # policy gains, reference and playback settings stay intact.
                preserved_config = ({key: value for key, value in config.items()
                                     if key not in RECOVERY_ENTRY_KEYS}
                                    if directory == PRONE_PACKAGE else config)
                serialized = json.dumps(preserved_config, sort_keys=True, separators=(",", ":"))
                self.assertEqual(hashlib.sha256(serialized.encode()).hexdigest(), expected_config)
                for name, expected in assets.items():
                    self.assertEqual(file_sha256(package / name), expected, name)
                asset_root = (package if config.get("method") == "supermimic" else
                              self.old if directory.startswith("old/") else CONFIG_ROOT)
                for key in ("policy_file", "reference_file", "trajectory_file_npz"):
                    if key in config:
                        self.assertTrue((asset_root / config[key]).is_file(), config[key])
                if directory in SUPER_PACKAGES | ARCHIVED_SUPER_PACKAGES:
                    self.assertEqual(config["hold_seconds"], 0.0)

    def test_prone_and_supine_share_pd_preparation_and_settle_conditions(self):
        prone = load_yaml(CONFIG_ROOT / PRONE_PACKAGE / "default.yaml")
        supine = load_yaml(CONFIG_ROOT / "rl_supine_to_stance_v1_hybrid_9843_m13000/default.yaml")
        for key in RECOVERY_ENTRY_KEYS:
            with self.subTest(parameter=key):
                self.assertIn(key, prone)
                self.assertEqual(prone[key], supine[key])
        self.assertTrue(prone["entry_transition_to_reference_pose"])
        self.assertEqual(prone["entry_transition_reference_pose_tolerance"], 0.80)
        self.assertEqual(prone["entry_transition_reference_pose_settle_cycles"], 5)
        self.assertEqual(prone["control_dt"] *
                         prone["entry_transition_reference_pose_settle_cycles"], 0.10)
        self.assertEqual(prone["hold_seconds"], 0)

    def test_formal_strikes_keep_live_policy_entry(self):
        for directory in (SUPER_PACKAGES | ARCHIVED_SUPER_PACKAGES) - {PRONE_PACKAGE}:
            with self.subTest(package=directory):
                config = load_yaml(CONFIG_ROOT / directory / "default.yaml")
                self.assertFalse(config.get("entry_transition_to_reference_pose", False))

    def test_prone_replacement_preserves_controls_transitions_and_other_recoveries(self):
        self.assertEqual(self.tasks["prone_to_stance"], {
            "motion": "prone_to_stance", "period": 0.02,
            "runner": [{"name": "rl_supermimic_runner", "enabled": True,
                        "param_tag": "rl_prone_to_stance"}],
            "manual_transition": ["passive", "pd_stand", "walk_leo"],
            "auto_transition": "walk_leo", "key": ["START", "CROSS_Y_RIGHT"],
        })
        prone = load_yaml(CONFIG_ROOT / PRONE_PACKAGE / "default.yaml")
        self.assertEqual(prone["task"], "prone")
        entries = {name for name, task in self.tasks.items()
                   if "prone_to_stance" in task.get("manual_transition", [])}
        self.assertEqual(entries, {"passive", "pd_stand", "pd_stand_x", "pd_stand_y"})
        for mode, entries in self.mode["mode"].items():
            with self.subTest(mode=mode):
                scopes = {entry["tag"]: entry["scope"] for entry in entries}
                self.assertEqual(scopes["rl_prone_to_stance"], PRONE_PACKAGE + "/default")
        recovery_transitions = {
            "supine_to_stance": (["passive", "pd_stand", "walk_leo"], "walk_leo"),
            "getup": (["passive", "pd_stand", "walk", "walk_impact", "walk_leo", "walk_leo_terrain"], "walk_leo"),
            "getup2": (["passive", "pd_stand", "walk", "walk_impact", "walk_leo", "walk_leo_terrain"], "walk_leo"),
            "stance_to_supine": (["passive", "pd_stand"], "passive"),
        }
        for motion, (manual, automatic) in recovery_transitions.items():
            with self.subTest(motion=motion):
                period, key, runner, tag = PRESERVED_MOTIONS[motion]
                self.assertEqual(self.tasks[motion], {
                    "motion": motion, "period": period, "key": key,
                    "runner": [{"name": runner, "enabled": True, "param_tag": tag}],
                    "manual_transition": manual, "auto_transition": automatic,
                })

    def test_reference_shapes_and_policy_history_contract(self):
        for directory in SUPER_PACKAGES | ARCHIVED_SUPER_PACKAGES:
            package = CONFIG_ROOT / directory
            config = load_yaml(package / "default.yaml")
            with self.subTest(package=directory):
                self.assertEqual(config["method"], "supermimic")
                self.assertEqual(config["robot"], "t800")
                self.assertEqual(config["control_dt"], 0.02)
                self.assertEqual(config["physics_dt"], 0.002)
                self.assertEqual(config["observation_dim"], 25 + 25 + 3 + 3 + 25 + 6)
                self.assertEqual(config["goal_dim"], 650)
                self.assertEqual(config["history_length"], 10)
                self.assertEqual(config["input_dim"], config["goal_dim"] +
                                 config["history_length"] * config["observation_dim"])
                self.assertEqual(config["output_dim"], 25)
                self.assertEqual(config["history_order"], "oldest_to_newest_frame_major")
                raw_action_export = (config["action_target_formula"] ==
                                     "default_joint_pos + action_scale * raw_action")
                action_name = "raw_action" if raw_action_export else "action_output"
                output_contract = config.get("action_output_contract")
                # The local-clip exports call their target variable raw_action,
                # but explicitly declare that the MNN wrapper returns executed
                # (clamped) actions. History follows that exported output.
                previous_action = ("previous_raw_action" if raw_action_export and not output_contract
                                   else "previous_executed_action")
                self.assertEqual(config["action_target_formula"],
                                 "default_joint_pos + action_scale * " + action_name)
                self.assertEqual(config["observation_layout"], [
                    "q-default_q:25", "joint_velocity:25", "body_angular_velocity:3",
                    "projected_gravity:3", previous_action + ":25", "relative_root_orientation_6d:6"])
                if not raw_action_export or output_contract:
                    self.assertIn(output_contract, (
                        "export wrapper returns executed_action(raw_action)=clamp(raw_action,-20,20)",
                        "clamp(raw_action,-20,20); previous action is executed action",
                        "executed_action(raw_action)=clamp(raw_action,-20,20)"))
                with np.load(package / config["reference_file"], allow_pickle=False) as reference:
                    frames = reference["joint_pos"].shape[0]
                    self.assertGreaterEqual(frames, 2)
                    shapes = {"joint_pos": (frames, 25), "joint_vel": (frames, 25),
                              "root_pos": (frames, 3), "root_quat": (frames, 4),
                              "root_rotation": (frames, 3, 3), "root_lin_vel": (frames, 3),
                              "root_ang_vel": (frames, 3), "goal": (frames, 650)}
                    for name, shape in shapes.items():
                        values = reference[name]
                        self.assertEqual(values.shape, shape, name)
                        self.assertEqual(values.dtype, np.dtype("float32"), name)
                        self.assertTrue(np.isfinite(values).all(), name)
                    np.testing.assert_allclose(np.linalg.norm(reference["root_quat"], axis=1), 1.0, atol=2e-4)
                    rotations = reference["root_rotation"]
                    np.testing.assert_allclose(rotations.transpose(0, 2, 1) @ rotations,
                        np.broadcast_to(np.eye(3), (frames, 3, 3)), atol=2e-4)

    def test_each_policy_joint_maps_to_one_sdk_joint_with_the_same_semantics(self):
        sdk_joints = [joint for limb in load_yaml(CONFIG_ROOT / "model/default.yaml")["limbs"]
                      for joint in limb["joints"]]
        self.assertEqual(len(sdk_joints), 25)
        self.assertEqual(len(set(sdk_joints)), 25)
        for directory in SUPER_PACKAGES | ARCHIVED_SUPER_PACKAGES:
            package = CONFIG_ROOT / directory
            config = load_yaml(package / "default.yaml")
            with self.subTest(package=directory):
                self.assertEqual(config["mujoco_joint_names"], sdk_joints)
                self.assertEqual(len(config["joint_names"]), 25)
                self.assertEqual(len(set(config["joint_names"])), 25)
                for policy_joint, sdk_joint in zip(config["joint_names"], sdk_joints):
                    # Numeric prefixes differ on the right arm and head. Match
                    # the complete joint role/side, rather than those prefixes.
                    self.assertEqual(policy_joint.split("_", 1)[1], sdk_joint.split("_", 1)[1])
                for name in ("default_joint_pos", "action_scale", "joint_stiffness", "joint_damping"):
                    values = np.asarray(config[name])
                    self.assertEqual(values.shape, (25,), name)
                    self.assertTrue(np.isfinite(values).all(), name)


if __name__ == "__main__":
    unittest.main()
