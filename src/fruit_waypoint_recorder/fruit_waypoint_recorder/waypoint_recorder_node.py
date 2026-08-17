"""ROS 2 node for recording the current map-to-base transform."""

from datetime import datetime
from pathlib import Path
from typing import Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from fruit_waypoint_recorder.recorder import (
    JoyEdgeDetector,
    WaypointStore,
    new_session_path,
)


DEFAULT_OUTPUT_DIRECTORY = "/home/rc4/fruit_ws/src/fruit_waypoint_recorder/waypoints"


class WaypointRecorderNode(Node):
    """Record base_link poses through a service or joystick controls."""

    def __init__(self) -> None:
        super().__init__("waypoint_recorder")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("output_directory", DEFAULT_OUTPUT_DIRECTORY)
        self.declare_parameter("save_button", 0)
        self.declare_parameter("undo_axis", 6)
        self.declare_parameter("undo_direction", 1.0)
        self.declare_parameter("axis_threshold", 0.5)
        self.declare_parameter("transform_timeout_sec", 1.0)

        joy_topic = self.get_parameter("joy_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value
        output_directory_value = self.get_parameter("output_directory").value
        save_button = self.get_parameter("save_button").value
        undo_axis = self.get_parameter("undo_axis").value
        undo_direction = self.get_parameter("undo_direction").value
        axis_threshold = self.get_parameter("axis_threshold").value
        self._transform_timeout_sec = self.get_parameter("transform_timeout_sec").value
        self._validate_parameters(
            joy_topic,
            output_directory_value,
            save_button,
            undo_axis,
            undo_direction,
            axis_threshold,
        )

        started_at = datetime.now().astimezone()
        output_directory = Path(output_directory_value)
        output_path = new_session_path(output_directory, started_at)
        self._store = WaypointStore(
            output_path,
            self._frame_id,
            self._child_frame_id,
            started_at,
        )
        self._edges = JoyEdgeDetector(
            save_button,
            undo_axis,
            undo_direction,
            axis_threshold,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._joy_subscription = self.create_subscription(
            Joy,
            joy_topic,
            self._on_joy,
            10,
        )
        self._save_service = self.create_service(
            Trigger,
            "~/save",
            self._on_save_service,
        )
        self.get_logger().info(f"Waypoint session file: {output_path}")
        self.get_logger().info(
            f"A/save button index {save_button}; D-pad undo axis {undo_axis} "
            f"direction {undo_direction:+.0f}"
        )

    def _validate_parameters(
        self,
        joy_topic: str,
        output_directory: str,
        save_button: int,
        undo_axis: int,
        undo_direction: float,
        axis_threshold: float,
    ) -> None:
        if not joy_topic or not self._frame_id or not self._child_frame_id:
            raise ValueError("joy_topic and TF frame IDs must not be empty")
        if not output_directory:
            raise ValueError("output_directory must not be empty")
        if save_button < 0 or undo_axis < 0:
            raise ValueError("joystick indexes must not be negative")
        if undo_direction not in (-1.0, 1.0):
            raise ValueError("undo_direction must be -1.0 or 1.0")
        if axis_threshold <= 0.0 or axis_threshold > 1.0:
            raise ValueError("axis_threshold must be in (0.0, 1.0]")
        if self._transform_timeout_sec <= 0.0:
            raise ValueError("transform_timeout_sec must be greater than zero")

    def _on_joy(self, message: Joy) -> None:
        save_edge, undo_edge = self._edges.observe(message.buttons, message.axes)
        if save_edge:
            success, result = self._save_current_pose()
            if success:
                self.get_logger().info(result)
            else:
                self.get_logger().error(result)
        if undo_edge:
            removed = self._store.undo()
            if removed is None:
                self.get_logger().warning("No waypoint to undo")
            else:
                self.get_logger().info(
                    f"Removed {removed['name']} from {self._store.output_path}"
                )

    def _on_save_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success, response.message = self._save_current_pose()
        return response

    def _save_current_pose(self) -> Tuple[bool, str]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._child_frame_id,
                Time(),
                timeout=Duration(seconds=self._transform_timeout_sec),
            )
        except TransformException as error:
            return False, (
                f"Cannot save waypoint: {self._frame_id} -> "
                f"{self._child_frame_id} unavailable: {error}"
            )

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        waypoint = self._store.append(
            translation={"x": translation.x, "y": translation.y, "z": translation.z},
            rotation={"x": rotation.x, "y": rotation.y, "z": rotation.z, "w": rotation.w},
            recorded_at=datetime.now().astimezone(),
        )
        return True, f"Saved {waypoint['name']} to {self._store.output_path}"


def main(args=None) -> None:
    """Run the waypoint recorder node."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = WaypointRecorderNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
