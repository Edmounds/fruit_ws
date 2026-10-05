"""ROS node entry point for the minimal fruit mission."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from fruit_mission.config import load_config
from fruit_mission.Logic import debug_a, debug_b, debug_c, main as competition_main, run_ac
from fruit_mission.mission_context import MissionContext


ENTRYPOINTS = {
    "full": competition_main,
    "a": debug_a,
    "b": debug_b,
    "c": debug_c,
    "ac": run_ac,
}


DEFAULT_CONFIG_PATH = (
    "/home/rc4/fruit_ws/src/fruit_mission/config/fruit_mission.yaml"
)


class FruitMissionNode(Node):
    """Start the readable A/B/C mission flow once."""

    def __init__(self) -> None:
        super().__init__("fruit_mission")
        self.declare_parameter("config_path", DEFAULT_CONFIG_PATH)
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("mission_flow", "full")
        self.declare_parameter("map_frame", "odom")
        config_path = self.get_parameter("config_path").value
        waypoint_file = str(self.get_parameter("waypoint_file").value).strip()
        mission_flow = str(self.get_parameter("mission_flow").value).strip().lower()
        map_frame = str(self.get_parameter("map_frame").value).strip()
        self.config = load_config(str(config_path), waypoint_file)
        self.config["mission_flow"] = mission_flow or "full"
        self.config["map_frame"] = map_frame
        try:
            self._entrypoint = ENTRYPOINTS[self.config["mission_flow"]]
        except KeyError as error:
            raise ValueError("mission_flow must be full, ac, a, b, or c") from error
        self._mission_context = MissionContext(self, self.config)

    def run(self) -> None:
        """Select once; startup/cleanup are not mixed into competition composition."""
        ctx = self._mission_context
        try:
            ctx.announce_and_wait("我们是重庆大学 RoboCup 水果采摘队伍，现在开始采摘")
            ctx.recover_home()
            self._entrypoint(ctx)
            ctx.announce_summary()
        finally:
            ctx.close_observation_window()

    def destroy_node(self) -> bool:
        """Stop mission helpers before destroying the ROS node."""
        mission_context = getattr(self, "_mission_context", None)
        if mission_context is not None:
            mission_context.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the fruit mission node."""
    rclpy.init(args=args)
    node = None
    try:
        node = FruitMissionNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
