# 启动基线（只读，不是修复计划）

启动前 Git diff（既有改动，不得擅自归因）：

diff --git a/src/fruit_arm_planner/config/arm_presets.yaml b/src/fruit_arm_planner/config/arm_presets.yaml
index 020a919..8b937a2 100644
--- a/src/fruit_arm_planner/config/arm_presets.yaml
+++ b/src/fruit_arm_planner/config/arm_presets.yaml
@@ -191,10 +191,30 @@ arm_planner_node:
         - {duration_ms: 2000, gripper: 0, target: [0.22, -0.00, 0.17, 0.0]}
 
       left_look_b:
-        description: "机械臂左侧观察位姿"
+        description: "机械臂左侧观察位姿,必须从伸直状态切换到这个状态"
         coordinate_frame: "cartesian"
         waypoints:
+          # - { target: [0.1202, 0.0000, 0.550, 0.0], duration_ms: 200, gripper: 0 }
           - { target: [0.0203, 0.0703, 0.303, -46.7], duration_ms: 1000, gripper: 0 }
-      #right_look_b:
-      #left_store_b:
-      #right_store_b
+
+      right_look_b:
+        description: "机械臂右侧观察位姿,必须从伸直状态切换到这个状态"
+        coordinate_frame: "cartesian"
+        waypoints:
+          # - { target: [0.1202, 0.0000, 0.550, 0.0], duration_ms: 200, gripper: 0 }
+          - { target: [0.0203, -0.0703, 0.303, 46.7], duration_ms: 1000, gripper: 0 }
+
+      left_store_b:
+        description: Recorded from keyboard jog mode
+        coordinate_frame: cartesian
+        waypoints:
+        - target:
+          - 0.022211213483941783
+          - 0.10852789656422027
+          - 0.21064164461799356
+          - -5.9204818979962575
+          duration_ms: 1000
+          gripper: 0
+        - {duration_ms: 1000, gripper: 1, target: [-0.1962624798929228, 0.00965640213802654, 0.2506981509144537, -5.973156743762218]}
+        - {duration_ms: 1000, gripper: 0, target: [0.042288450212795675, 0.15905864173713108, 0.3536682379281416, -4.166016328536959]}
+        - { target: [0.1202, 0.0000, 0.550, 0.0], duration_ms: 1000, gripper: 0 }
diff --git a/src/fruit_mission/config/fruit_mission.yaml b/src/fruit_mission/config/fruit_mission.yaml
index 4b240bd..c0efb4b 100644
--- a/src/fruit_mission/config/fruit_mission.yaml
+++ b/src/fruit_mission/config/fruit_mission.yaml
@@ -28,7 +28,7 @@ fruit_mission:
     # Local +X is gripper forward, +Y is gripper left, +Z is gripper up.
     # Applied relative to arm end-effector orientation before MoveArmCartesian.
     target_offset_ac_m: [0.0, 0.03, -0.015]
-    target_offset_b_m: [0.03, 0.0, -0.015]
+    target_offset_b_m: [0.026, 0.0, -0.01]
 
     tts:
       manifest_path: "/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml"

启动前 status：
 M src/fruit_arm_planner/config/arm_presets.yaml
 M src/fruit_mission/config/fruit_mission.yaml
?? .agents/skills/wisadel-helium-voice/SKILL.md
?? .pi/skills/wisadel-helium-voice/SKILL.md
?? ce1adf5a259fbc6f8ffcf4c76b95ef9c.png
?? image.png

