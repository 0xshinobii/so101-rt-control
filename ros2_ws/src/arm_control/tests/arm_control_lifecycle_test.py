"""Lifecycle integration test for arm_control_node.

Drives configure, activate, deactivate, activate, deactivate, cleanup for 20
cycles. Worker counts come from the control thread, and freshness comes from
the plant sample time. The last step leaves the node active so the launch
harness is what interrupts it.
"""

import os
import unittest

from arm_msgs.msg import ArmMetrics
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import Node
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node as RclpyNode

NODE_NAME = "arm_control_node"
BAD_NODE_NAME = "arm_control_bad_model"
CYCLES = 20
QUIET_WINDOW_SEC = 0.30
OBSERVE_WINDOW_SEC = 0.40
CONFIGURE_TIMEOUT_SEC = 15.0
TRANSITION_TIMEOUT_SEC = 2.0

CYCLE = (
    (Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE, CONFIGURE_TIMEOUT_SEC),
    (Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE, TRANSITION_TIMEOUT_SEC),
    (Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE, TRANSITION_TIMEOUT_SEC),
    (Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE, TRANSITION_TIMEOUT_SEC),
    (Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE, TRANSITION_TIMEOUT_SEC),
    (Transition.TRANSITION_CLEANUP, State.PRIMARY_STATE_UNCONFIGURED, TRANSITION_TIMEOUT_SEC),
)


def _params_file():
    return os.path.normpath(os.path.join(
        os.path.dirname(__file__),
        "..", "..", "arm_bringup", "config", "params.yaml",
    ))


@pytest.mark.launch_test
def generate_test_description():
    params = _params_file()
    dut = Node(
        package="arm_control",
        executable="arm_control_node",
        name=NODE_NAME,
        output="screen",
        parameters=[params, {
            "rt_enable": False,
            "jitter_samples": 0,
            "worker_counters": True,
        }],
    )
    bad = Node(
        package="arm_control",
        executable="arm_control_node",
        name=BAD_NODE_NAME,
        output="screen",
        parameters=[params, {
            "rt_enable": False,
            "jitter_samples": 0,
            "model_path": "/no/such/model.xml",
        }],
    )
    return launch.LaunchDescription([
        dut,
        bad,
        launch_testing.actions.ReadyToTest(),
    ])


class _Client(RclpyNode):
    def __init__(self, node_name):
        super().__init__(f"{node_name}_lifecycle_test")
        self._node_name = node_name
        self._change = self.create_client(
            ChangeState, f"/{node_name}/change_state"
        )
        self._get = self.create_client(GetState, f"/{node_name}/get_state")
        self._params = self.create_client(
            GetParameters, f"/{node_name}/get_parameters"
        )
        self.sample_times = []
        if node_name == NODE_NAME:
            self.create_subscription(ArmMetrics, "arm_metrics", self._on_metrics, 10)

    def _on_metrics(self, msg):
        self.sample_times.append(msg.sample_time)

    def wait_ready(self):
        if not self._change.wait_for_service(timeout_sec=CONFIGURE_TIMEOUT_SEC):
            raise AssertionError(f"{self._node_name} change_state did not appear")
        if not self._get.wait_for_service(timeout_sec=CONFIGURE_TIMEOUT_SEC):
            raise AssertionError(f"{self._node_name} get_state did not appear")
        if not self._params.wait_for_service(timeout_sec=CONFIGURE_TIMEOUT_SEC):
            raise AssertionError(f"{self._node_name} get_parameters did not appear")

    def state_id(self):
        future = self._get.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=TRANSITION_TIMEOUT_SEC
        )
        if not future.done() or future.result() is None:
            raise AssertionError(f"{self._node_name} get_state hung or returned nothing")
        return future.result().current_state.id

    def transition(self, transition_id, expected_state, timeout_sec, expect_success=True):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self._change.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise AssertionError(
                f"{self._node_name} transition {transition_id} hung past {timeout_sec}s"
            )
        result = future.result()
        if result is None:
            raise AssertionError(f"{self._node_name} transition {transition_id} returned nothing")
        if result.success != expect_success:
            raise AssertionError(
                f"{self._node_name} transition {transition_id} "
                f"success={result.success}, expected {expect_success}"
            )
        state_id = self.state_id()
        if state_id != expected_state:
            raise AssertionError(
                f"{self._node_name} state {state_id} after transition {transition_id}, "
                f"expected {expected_state}"
            )

    def spin_for(self, duration_sec):
        end = self.get_clock().now() + rclpy.duration.Duration(seconds=duration_sec)
        while self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def await_sample(self, timeout_sec):
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        start = len(self.sample_times)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.sample_times) > start:
                return self.sample_times[-1]
        raise AssertionError("no arm_metrics while active")

    def count_new_samples(self, duration_sec):
        start = len(self.sample_times)
        self.spin_for(duration_sec)
        return len(self.sample_times) - start

    def worker_counts(self):
        request = GetParameters.Request()
        request.names = ["active_workers", "max_workers"]
        future = self._params.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=TRANSITION_TIMEOUT_SEC
        )
        if not future.done() or future.result() is None:
            raise AssertionError("get_parameters hung or returned nothing")
        values = future.result().values
        if len(values) != 2:
            raise AssertionError(f"expected 2 worker counters, got {len(values)}")
        return values[0].integer_value, values[1].integer_value


class TestArmControlLifecycle(unittest.TestCase):
    def test_repeated_cycles_then_leave_active(self):
        rclpy.init()
        client = _Client(NODE_NAME)
        bad = _Client(BAD_NODE_NAME)
        try:
            client.wait_ready()
            bad.wait_ready()
            if client.state_id() != State.PRIMARY_STATE_UNCONFIGURED:
                raise AssertionError("node did not start unconfigured")
            bad.transition(
                Transition.TRANSITION_CONFIGURE,
                State.PRIMARY_STATE_UNCONFIGURED,
                CONFIGURE_TIMEOUT_SEC,
                expect_success=False,
            )
            if bad.state_id() != State.PRIMARY_STATE_UNCONFIGURED:
                raise AssertionError("failed configure did not leave the node unconfigured")
            for cycle in range(CYCLES):
                self._run_cycle(client, cycle)
            self._leave_active(client)
        finally:
            client.destroy_node()
            bad.destroy_node()
            rclpy.shutdown()

    def _run_cycle(self, client, cycle):
        previous_sample = None
        for transition_id, expected_state, timeout_sec in CYCLE:
            if transition_id == Transition.TRANSITION_CONFIGURE:
                client.transition(transition_id, expected_state, timeout_sec)
                if client.count_new_samples(QUIET_WINDOW_SEC) != 0:
                    raise AssertionError(
                        f"cycle {cycle}: telemetry published while inactive"
                    )
            elif transition_id == Transition.TRANSITION_ACTIVATE:
                client.transition(transition_id, expected_state, timeout_sec)
                first = client.await_sample(TRANSITION_TIMEOUT_SEC)
                client.spin_for(OBSERVE_WINDOW_SEC)
                latest = client.sample_times[-1]
                if latest <= first:
                    raise AssertionError(
                        f"cycle {cycle}: sample_time did not advance "
                        f"({first} -> {latest})"
                    )
                if previous_sample is not None and first <= previous_sample:
                    raise AssertionError(
                        f"cycle {cycle}: sample_time {first} is not newer than "
                        f"{previous_sample}"
                    )
                active, peak = client.worker_counts()
                if active != 1 or peak != 1:
                    raise AssertionError(
                        f"cycle {cycle}: active_workers={active} max_workers={peak}"
                    )
                previous_sample = latest
            elif transition_id == Transition.TRANSITION_DEACTIVATE:
                client.transition(transition_id, expected_state, timeout_sec)
                active, _peak = client.worker_counts()
                if active != 0:
                    raise AssertionError(
                        f"cycle {cycle}: active_workers={active} after deactivate"
                    )
                client.spin_for(QUIET_WINDOW_SEC)
                held = len(client.sample_times)
                if client.count_new_samples(QUIET_WINDOW_SEC) != 0 or len(client.sample_times) != held:
                    raise AssertionError(
                        f"cycle {cycle}: telemetry continued after deactivate"
                    )
            else:
                client.transition(transition_id, expected_state, timeout_sec)

    def _leave_active(self, client):
        client.transition(
            Transition.TRANSITION_CONFIGURE,
            State.PRIMARY_STATE_INACTIVE,
            CONFIGURE_TIMEOUT_SEC,
        )
        if client.count_new_samples(QUIET_WINDOW_SEC) != 0:
            raise AssertionError("telemetry published while inactive")
        client.transition(
            Transition.TRANSITION_ACTIVATE,
            State.PRIMARY_STATE_ACTIVE,
            TRANSITION_TIMEOUT_SEC,
        )
        first = client.await_sample(TRANSITION_TIMEOUT_SEC)
        client.spin_for(OBSERVE_WINDOW_SEC)
        if client.sample_times[-1] <= first:
            raise AssertionError("sample_time did not advance while active")
        active, peak = client.worker_counts()
        if active != 1 or peak != 1:
            raise AssertionError(f"active_workers={active} max_workers={peak}")


@launch_testing.post_shutdown_test()
class TestArmControlExit(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
