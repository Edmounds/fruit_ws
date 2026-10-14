"""Observational mission status, with no motion or flow decisions."""

import json
import time

from std_msgs.msg import String

from fruit_mission.competition_protocol import count_result
from fruit_mission.script_hardware_client import latched_qos


class MissionTelemetry:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(String, '/fruit_mission/status', latched_qos())
        self.data = {'phase': 'RUNNING', 'area': '', 'target': '', 'step': 'starting',
                     'statistics': {}, 'events': []}

    def step(self, step, area=None, target=None):
        self.data['step'] = step
        if area is not None:
            self.data['area'] = area
        if target is not None:
            self.data['target'] = str(target)
        self.publish()

    def result(self, outcome, reason, model_type=None):
        count_result(self.data['statistics'], self.data['area'], model_type, outcome)
        self.data['events'] = [*self.data['events'][-49:], {
            'at': time.time(), 'area': self.data['area'], 'target': self.data['target'],
            'outcome': outcome, 'reason': reason, 'type': model_type or 'unknown'}]
        self.publish()

    def publish(self):
        self.data['updated_at'] = time.time()
        try:
            self.publisher.publish(String(data=json.dumps(self.data, ensure_ascii=False)))
        except Exception as error:
            self.node.get_logger().warning(f'Mission telemetry unavailable: {error}')
