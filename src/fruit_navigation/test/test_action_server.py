import threading
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.action import GoalResponse
from rclpy.clock import ClockType
from rclpy.time import Time

from fruit_navigation.axis_waypoint_action_server import AxisWaypointActionServer


class FakeLogger:
    def error(self, _message):
        pass


class FakeClock:
    def __init__(self, seconds):
        self._now = Time(seconds=seconds, clock_type=ClockType.ROS_TIME)

    def now(self):
        return self._now


class FakeTransformBuffer:
    def __init__(self, transform):
        self._transform = transform

    def lookup_transform(self, *_args, **_kwargs):
        return self._transform


class FakeGoalHandle:
    def __init__(self, active=True):
        self.request = SimpleNamespace(poses=[PoseStamped()])
        self.is_active = active
        self.abort_count = 0
        self.succeed_count = 0
        self.feedback = []

    def publish_feedback(self, message):
        self.feedback.append(message.current_waypoint)

    def abort(self):
        self.abort_count += 1
        self.is_active = False

    def succeed(self):
        self.succeed_count += 1
        self.is_active = False


def bare_server(localization_ready=True):
    server = AxisWaypointActionServer.__new__(AxisWaypointActionServer)
    server._map_frame = "map"
    server._active_goal = False
    server._goal_lock = threading.Lock()
    server._localization_ready = localization_ready
    server._localization_error = False
    server.get_logger = lambda: FakeLogger()
    server._publish_stop = lambda: None
    server._publish_stop_burst = lambda: None
    return server


def valid_goal():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    return SimpleNamespace(poses=[pose])


def diagnostic(level, accepted, has_accepted_once):
    status = DiagnosticStatus()
    status.name = "pb_gicp_localization/registration"
    status.level = level
    status.values = [
        KeyValue(key="accepted", value=accepted),
        KeyValue(key="has_accepted_once", value=has_accepted_once),
    ]
    return DiagnosticArray(status=[status])


def test_goal_is_rejected_until_gicp_has_accepted_once():
    server = bare_server(localization_ready=False)

    assert server._on_goal(valid_goal()) == GoalResponse.REJECT

    server._on_diagnostics(diagnostic(DiagnosticStatus.WARN, "0", "0"))
    assert server._on_goal(valid_goal()) == GoalResponse.REJECT

    server._on_diagnostics(diagnostic(DiagnosticStatus.OK, "1", "1"))
    assert server._on_goal(valid_goal()) == GoalResponse.ACCEPT


def test_second_goal_is_rejected_as_soon_as_first_goal_is_accepted():
    server = bare_server()

    assert server._on_goal(valid_goal()) == GoalResponse.ACCEPT
    assert server._on_goal(valid_goal()) == GoalResponse.REJECT


def test_execute_does_not_abort_a_goal_that_is_already_terminal():
    server = bare_server()
    server._active_goal = True
    server._follow_one = lambda _goal_handle, _target: False
    goal_handle = FakeGoalHandle(active=False)

    result = server._execute(goal_handle)

    assert list(result.missed_waypoints) == [0]
    assert goal_handle.abort_count == 0
    assert server._active_goal is False


def test_execute_succeeds_and_releases_goal_reservation():
    server = bare_server()
    server._active_goal = True
    server._follow_one = lambda _goal_handle, _target: True
    goal_handle = FakeGoalHandle()

    result = server._execute(goal_handle)

    assert list(result.missed_waypoints) == []
    assert goal_handle.succeed_count == 1
    assert server._active_goal is False


def test_lookup_pose_rejects_a_transform_older_than_tf_timeout():
    server = bare_server()
    transform = TransformStamped()
    transform.header.stamp.sec = 9
    transform.transform.rotation.w = 1.0
    server._tf_buffer = FakeTransformBuffer(transform)
    server._base_frame = "base_link"
    server._tf_timeout_sec = 0.3
    server.get_clock = lambda: FakeClock(10.0)
    stop_bursts = []
    server._publish_stop_burst = lambda: stop_bursts.append(True)

    assert server._lookup_pose() is None
    assert stop_bursts == [True]
