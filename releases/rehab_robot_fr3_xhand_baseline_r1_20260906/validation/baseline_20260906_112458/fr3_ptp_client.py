from __future__ import annotations

from rclpy.action import ActionClient

from franka_msgs.action import PTPMotion

from .fr3_ptp_contract import (
    FR3PTPContractError,
    PTP_ACTION_TYPE,
    build_goal,
    discover_unique_ptp_action,
)


class FR3PTPExecutionGateError(RuntimeError):
    pass


def discover_from_node(node):
    graph = node.get_action_names_and_types()

    return discover_unique_ptp_action(
        graph
    )


class FR3PTPClient:
    """
    Real ActionClient wrapper.

    Merely constructing this object does not send a goal.
    """

    def __init__(
        self,
        node,
        action_name: str,
    ):
        if not action_name:
            raise FR3PTPContractError(
                "PTP action name must not be empty"
            )

        self._node = node
        self._action_name = action_name

        self._client = ActionClient(
            node,
            PTPMotion,
            action_name,
        )

    @property
    def action_name(self):
        return self._action_name

    def wait_ready(self, timeout_sec=2.0):
        return self._client.wait_for_server(
            timeout_sec=float(timeout_sec)
        )

    def build_goal(
        self,
        target,
        *,
        maximum_velocity_rad_s=0.020,
        goal_tolerance=0.001,
    ):
        return build_goal(
            target,
            maximum_velocity_rad_s=(
                maximum_velocity_rad_s
            ),
            goal_tolerance=goal_tolerance,
        )

    def send_goal_async(
        self,
        goal,
        *,
        execute=False,
    ):
        if not execute:
            raise FR3PTPExecutionGateError(
                "PTP goal blocked: execute flag is false"
            )

        return self._client.send_goal_async(
            goal
        )
