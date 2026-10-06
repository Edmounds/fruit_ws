# 启动基线（只读，不是修复计划）

启动前 Git diff（既有改动，不得擅自归因）：

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
 M src/fruit_mission/config/fruit_mission.yaml
?? .agents/skills/wisadel-helium-voice/SKILL.md
?? .pi/skills/wisadel-helium-voice/SKILL.md
?? ce1adf5a259fbc6f8ffcf4c76b95ef9c.png
?? image.png

