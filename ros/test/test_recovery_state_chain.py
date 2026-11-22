import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_system_interfaces.msg import LocalizationHealth, RecoveryStatus, SystemMode
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    manager = launch_ros.actions.Node(
        package="kiss_matcher_ros",
        executable="system_mode_manager",
        name="system_mode_manager_integration_test",
        parameters=[{"map_directory": "/tmp/recovery-state-chain-map"}],
        output="screen",
    )
    return launch.LaunchDescription(
        [manager, launch_testing.actions.ReadyToTest()]
    ), {"manager": manager}


class TestRecoveryStateChain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("recovery_state_chain_probe")
        self.latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.recovery = None
        self.mode = None
        self.health_pub = self.node.create_publisher(
            LocalizationHealth, "/localization/health", self.latched_qos
        )
        self.localized_pub = self.node.create_publisher(
            Bool, "/relocalization/localized", self.latched_qos
        )
        self.recovery_sub = self.node.create_subscription(
            RecoveryStatus,
            "/recovery/state",
            lambda message: setattr(self, "recovery", message),
            self.latched_qos,
        )
        self.mode_sub = self.node.create_subscription(
            SystemMode,
            "/system_mode/state",
            lambda message: setattr(self, "mode", message),
            self.latched_qos,
        )
        self.ack_client = self.node.create_client(Trigger, "/recovery/acknowledge")

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail("timed out waiting for recovery state transition")

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def publish_bad_health(self):
        message = LocalizationHealth()
        message.state = LocalizationHealth.BAD
        message.effective_features = 0
        message.mean_residual_m = 1.0
        message.queue_delay_ms = 2200.0
        message.reason = "integration test LiDAR gap"
        self.health_pub.publish(message)

    def test_bad_relocalize_confirm_resume_and_late_joiner(self):
        self.spin_until(
            lambda: self.recovery is not None
            and self.recovery.state == RecoveryStatus.NORMAL
            and self.mode is not None
            and self.mode.mode == SystemMode.NORMAL
        )
        self.spin_until(lambda: self.health_pub.get_subscription_count() > 0)

        self.publish_bad_health()
        self.spin_for(0.25)
        self.publish_bad_health()
        self.spin_until(
            lambda: self.recovery.state == RecoveryStatus.RECOVERING
            and not self.recovery.map_writes_allowed
            and not self.recovery.autonomy_allowed
            and self.mode.mode == SystemMode.RELOCALIZATION
        )

        localized = Bool()
        localized.data = True
        self.localized_pub.publish(localized)
        self.spin_until(
            lambda: self.recovery.state
            == RecoveryStatus.RECOVERED_WAITING_CONFIRMATION
            and self.mode.mode == SystemMode.NORMAL
        )
        self.assertTrue(self.recovery.motion_allowed)
        self.assertFalse(self.recovery.map_writes_allowed)
        self.assertFalse(self.recovery.autonomy_allowed)

        self.assertTrue(self.ack_client.wait_for_service(timeout_sec=5.0))
        future = self.ack_client.call_async(Trigger.Request())
        self.spin_until(lambda: future.done())
        self.assertTrue(future.result().success)
        self.spin_until(
            lambda: self.recovery.state == RecoveryStatus.NORMAL
            and self.recovery.map_writes_allowed
            and self.recovery.autonomy_allowed
        )

        late_state = []
        late_sub = self.node.create_subscription(
            RecoveryStatus,
            "/recovery/state",
            late_state.append,
            self.latched_qos,
        )
        self.spin_until(lambda: len(late_state) == 1)
        self.assertEqual(late_state[0].state, RecoveryStatus.NORMAL)
        self.assertTrue(late_state[0].map_writes_allowed)
        self.node.destroy_subscription(late_sub)
