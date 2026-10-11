"""Lifecycle integration test for arm_control_node.

Drives configure, activate, deactivate, activate, deactivate, cleanup for 20
cycles. Worker counts come from the control thread, and freshness comes from
the plant sample time, which restarts after each deactivate. The last step
leaves the node active so the launch harness is what interrupts it.
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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.node import Node as RclpyNode

NODE_NAME = "arm_control_node"
BAD_NODE_NAME = "arm_control_bad_model"
FAULT_NODE_NAME = "arm_control_fault_test"
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
        "..", "..", "arm_bringup", "config", "sim.yaml",
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
    fault = Node(
        package="arm_control",
        executable="arm_control_node",
        name=FAULT_NODE_NAME,
        output="screen",
        parameters=[params, {
            "is_sim": False,
            "rt_enable": False,
            "jitter_samples": 0,
            "worker_counters": True,
            "hardware.test_bus": True,
            "hardware.port": "scripted",
            "hardware.home_duration": 0.02,
        }],
    )
    return launch.LaunchDescription([
        dut,
        bad,
        fault,
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
        self._set_params = self.create_client(
            SetParametersAtomically, f"/{node_name}/set_parameters_atomically"
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
        if not self._set_params.wait_for_service(timeout_sec=CONFIGURE_TIMEOUT_SEC):
            raise AssertionError(f"{self._node_name} set_parameters did not appear")

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
        values = self.int_parameters(["active_workers", "max_workers"])
        return values["active_workers"], values["max_workers"]

    def int_parameters(self, names):
        request = GetParameters.Request()
        request.names = names
        future = self._params.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=TRANSITION_TIMEOUT_SEC
        )
        if not future.done() or future.result() is None:
            raise AssertionError("get_parameters hung or returned nothing")
        values = future.result().values
        if len(values) != len(names):
            raise AssertionError(f"expected {len(names)} parameters, got {len(values)}")
        return {name: value.integer_value for name, value in zip(names, values)}

    def set_test_fault(self, mode, failure_limit=0):
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(
                name="test_bus.injection_mode",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=mode,
                ),
            ),
            Parameter(
                name="test_bus.failure_limit",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=failure_limit,
                ),
            ),
        ]
        future = self._set_params.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=TRANSITION_TIMEOUT_SEC
        )
        if not future.done() or future.result() is None:
            raise AssertionError("set_parameters hung or returned nothing")
        result = future.result().result
        if not result.successful:
            raise AssertionError(f"fault injection rejected: {result.reason}")

    def await_condition(self, predicate, timeout_sec, description):
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f"timed out waiting for {description}")


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
                if previous_sample is not None and first >= previous_sample:
                    raise AssertionError(
                        f"cycle {cycle}: sample_time {first} did not restart "
                        f"after deactivate (previous {previous_sample})"
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

    def test_scripted_bus_faults_and_recovery(self):
        rclpy.init()
        client = _Client(FAULT_NODE_NAME)
        stat_names = [
            "test_bus.failed_reads",
            "test_bus.failed_command_writes",
            "test_bus.successful_reads",
            "test_bus.successful_command_writes",
            "test_bus.torque_off_attempts",
            "test_bus.torque_off_failures",
        ]

        def stats():
            return client.int_parameters(stat_names)

        def configure_and_activate():
            client.transition(
                Transition.TRANSITION_CONFIGURE,
                State.PRIMARY_STATE_INACTIVE,
                CONFIGURE_TIMEOUT_SEC,
            )
            client.transition(
                Transition.TRANSITION_ACTIVATE,
                State.PRIMARY_STATE_ACTIVE,
                TRANSITION_TIMEOUT_SEC,
            )
            client.await_condition(
                lambda: stats()["test_bus.successful_reads"] >= 5,
                TRANSITION_TIMEOUT_SEC,
                "five successful tracking reads",
            )

        def assert_fault_latched():
            active, _peak = client.worker_counts()
            self.assertEqual(active, 0)
            client.transition(
                Transition.TRANSITION_ACTIVATE,
                State.PRIMARY_STATE_INACTIVE,
                TRANSITION_TIMEOUT_SEC,
                expect_success=False,
            )

        try:
            client.wait_ready()

            # Two stale-state cycles recover, and both still deliver commands.
            configure_and_activate()
            before = stats()
            client.set_test_fault("read_timeout", failure_limit=2)
            client.await_condition(
                lambda: stats()["test_bus.failed_reads"] == 2,
                TRANSITION_TIMEOUT_SEC,
                "two transient read timeouts",
            )
            client.await_condition(
                lambda: stats()["test_bus.successful_reads"]
                > before["test_bus.successful_reads"],
                TRANSITION_TIMEOUT_SEC,
                "read recovery",
            )
            recovered = stats()
            self.assertGreaterEqual(
                recovered["test_bus.successful_command_writes"],
                before["test_bus.successful_command_writes"] + 2,
            )
            self.assertEqual(client.state_id(), State.PRIMARY_STATE_ACTIVE)
            self.assertEqual(client.worker_counts()[0], 1)

            # Three consecutive read failures stop the worker. Torque-off works
            # because writes remain available in read-only timeout mode.
            client.set_test_fault("none")
            client.set_test_fault("read_timeout")
            client.await_condition(
                lambda: client.state_id() == State.PRIMARY_STATE_INACTIVE,
                TRANSITION_TIMEOUT_SEC,
                "inactive state after read fault",
            )
            read_fault = stats()
            self.assertEqual(read_fault["test_bus.failed_reads"], 5)
            self.assertEqual(read_fault["test_bus.failed_command_writes"], 0)
            self.assertEqual(read_fault["test_bus.torque_off_attempts"], 6)
            self.assertEqual(read_fault["test_bus.torque_off_failures"], 0)
            assert_fault_latched()

            client.set_test_fault("none")
            client.transition(
                Transition.TRANSITION_CLEANUP,
                State.PRIMARY_STATE_UNCONFIGURED,
                TRANSITION_TIMEOUT_SEC,
            )

            # Combined timeouts produce three failed reads, two skipped command
            # cycles with failed writes, and six separately reported torque-off
            # failures.
            configure_and_activate()
            client.set_test_fault("read_write_timeout")
            client.await_condition(
                lambda: client.state_id() == State.PRIMARY_STATE_INACTIVE,
                TRANSITION_TIMEOUT_SEC,
                "inactive state after combined fault",
            )
            combined = stats()
            self.assertEqual(combined["test_bus.failed_reads"], 3)
            self.assertEqual(combined["test_bus.failed_command_writes"], 2)
            self.assertEqual(combined["test_bus.torque_off_attempts"], 6)
            self.assertEqual(combined["test_bus.torque_off_failures"], 6)
            successful_writes = combined["test_bus.successful_command_writes"]
            client.spin_for(0.10)
            self.assertEqual(
                stats()["test_bus.successful_command_writes"], successful_writes
            )
            assert_fault_latched()

            # Deliberate recovery creates exactly one replacement worker.
            client.set_test_fault("none")
            client.transition(
                Transition.TRANSITION_CLEANUP,
                State.PRIMARY_STATE_UNCONFIGURED,
                TRANSITION_TIMEOUT_SEC,
            )
            configure_and_activate()
            self.assertEqual(client.worker_counts(), (1, 1))
            client.transition(
                Transition.TRANSITION_DEACTIVATE,
                State.PRIMARY_STATE_INACTIVE,
                TRANSITION_TIMEOUT_SEC,
            )
            client.transition(
                Transition.TRANSITION_CLEANUP,
                State.PRIMARY_STATE_UNCONFIGURED,
                TRANSITION_TIMEOUT_SEC,
            )
        finally:
            client.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestArmControlExit(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
