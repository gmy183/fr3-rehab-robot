from __future__ import annotations

from dataclasses import dataclass
import re
import subprocess
import time
from typing import Any, Dict, List, Tuple

from .config import V3Config


@dataclass
class SafetyReport:
    ok: bool
    blockers: List[str]
    data: Dict[str, Any]


class SafetySupervisor:
    """Read-only V3 safety supervisor.

    No FR3/XHand command is sent here.

    V3.1.1:
      - transient ROS topic read retry
      - multiple QoS read strategies
      - state-unavailable vs robot-fault distinction
      - topic diagnostics captured automatically
    """

    ROBOT_STATE_ATTEMPTS = 2
    ROBOT_STATE_TIMEOUT_S = 4.0
    ROBOT_STATE_RETRY_DELAY_S = 0.15

    def __init__(self, config: V3Config):
        self.config = config

    @staticmethod
    def _run(
        args: List[str],
        timeout: float = 5.0,
    ) -> Tuple[int, str]:

        try:
            p = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                check=False,
            )

            return p.returncode, p.stdout

        except subprocess.TimeoutExpired as exc:

            out = exc.stdout or ""

            if isinstance(out, bytes):
                out = out.decode(
                    errors="replace"
                )

            return 124, out + "\n[TIMEOUT]"

    @staticmethod
    def _contains_robot_state(text: str) -> bool:
        return (
            re.search(
                r"^robot_mode:\s*\d+",
                text,
                re.MULTILINE,
            )
            is not None
        )

    def _read_robot_state(
        self,
        topic: str,
    ) -> Tuple[bool, str, List[Dict[str, Any]]]:

        # Different Jazzy/DDS environments can behave differently
        # with ros2 topic echo QoS. Try compatible strategies
        # automatically instead of failing after one CLI call.

        strategies = [
            (
                "default",
                [
                    "ros2",
                    "topic",
                    "echo",
                    topic,
                    "--once",
                ],
            ),
            (
                "sensor_data",
                [
                    "ros2",
                    "topic",
                    "echo",
                    topic,
                    "--once",
                    "--qos-profile",
                    "sensor_data",
                ],
            ),
            (
                "best_effort",
                [
                    "ros2",
                    "topic",
                    "echo",
                    topic,
                    "--once",
                    "--qos-reliability",
                    "best_effort",
                    "--qos-durability",
                    "volatile",
                ],
            ),
        ]

        diagnostics: List[Dict[str, Any]] = []

        last_output = ""

        for outer in range(
            1,
            self.ROBOT_STATE_ATTEMPTS + 1,
        ):

            for name, command in strategies:

                rc, output = self._run(
                    command,
                    timeout=self.ROBOT_STATE_TIMEOUT_S,
                )

                last_output = output

                received = (
                    self._contains_robot_state(
                        output
                    )
                )

                diagnostics.append(
                    {
                        "attempt": outer,
                        "strategy": name,
                        "return_code": rc,
                        "received": received,
                    }
                )

                if received:

                    if (
                        outer > 1
                        or name != "default"
                    ):
                        print(
                            "[PASS] robot_state recovered "
                            f"using {name} "
                            f"(attempt {outer})"
                        )

                    return (
                        True,
                        output,
                        diagnostics,
                    )

            if (
                outer
                < self.ROBOT_STATE_ATTEMPTS
            ):
                print(
                    "[WAIT] robot_state unavailable; "
                    f"retry {outer}/"
                    f"{self.ROBOT_STATE_ATTEMPTS}"
                )

                time.sleep(
                    self.ROBOT_STATE_RETRY_DELAY_S
                )

        return (
            False,
            last_output,
            diagnostics,
        )

    def check_runtime(
        self,
        require_idle: bool = True,
        require_no_motion_controller: bool = True,
    ) -> SafetyReport:

        blockers: List[str] = []
        data: Dict[str, Any] = {}

        # ----------------------------------------------------
        # Controller manager
        # ----------------------------------------------------

        rc, controllers = self._run(
            [
                "ros2",
                "control",
                "list_controllers",
            ],
            timeout=5.0,
        )

        data["controllers_rc"] = rc
        data["controllers_raw"] = controllers

        if rc != 0:
            blockers.append(
                "CONTROLLER_STATE_UNAVAILABLE: "
                "controller_manager query failed"
            )

        active = []

        for line in controllers.splitlines():

            parts = line.split()

            if (
                len(parts) >= 3
                and parts[-1] == "active"
            ):
                active.append(
                    parts[0]
                )

        data["active_controllers"] = active

        allowed = {
            "joint_state_broadcaster",
            "franka_robot_state_broadcaster",
        }

        unexpected = [
            name
            for name in active
            if name not in allowed
        ]

        data[
            "unexpected_active_controllers"
        ] = unexpected

        if (
            require_no_motion_controller
            and unexpected
        ):
            blockers.append(
                "MOTION_CONTROLLER_ACTIVE: "
                f"{unexpected}"
            )

        # ----------------------------------------------------
        # Franka robot state
        # ----------------------------------------------------

        topic = str(
            self.config.hardware[
                "fr3"
            ]["robot_state_topic"]
        )

        received, state, attempts = (
            self._read_robot_state(
                topic
            )
        )

        data[
            "robot_state_read_attempts"
        ] = attempts

        data[
            "robot_state_raw"
        ] = state

        if not received:

            blockers.append(
                "STATE_UNAVAILABLE: "
                "robot_state not received"
            )

            # Automatically collect topic diagnostics.
            info_rc, info = self._run(
                [
                    "ros2",
                    "topic",
                    "info",
                    topic,
                    "-v",
                ],
                timeout=5.0,
            )

            data[
                "robot_state_topic_info_rc"
            ] = info_rc

            data[
                "robot_state_topic_info"
            ] = info

            type_rc, topic_type = self._run(
                [
                    "ros2",
                    "topic",
                    "type",
                    topic,
                ],
                timeout=4.0,
            )

            data[
                "robot_state_topic_type_rc"
            ] = type_rc

            data[
                "robot_state_topic_type"
            ] = topic_type

        else:

            mode_match = re.search(
                r"^robot_mode:\s*(\d+)",
                state,
                re.MULTILINE,
            )

            robot_mode = (
                int(mode_match.group(1))
                if mode_match
                else None
            )

            data[
                "robot_mode"
            ] = robot_mode

            required_mode = int(
                self.config.hardware[
                    "fr3"
                ].get(
                    "required_idle_robot_mode",
                    1,
                )
            )

            if (
                require_idle
                and robot_mode
                != required_mode
            ):
                blockers.append(
                    "ROBOT_NOT_IDLE: "
                    f"robot_mode={robot_mode}, "
                    f"required={required_mode}"
                )

            bad_flags = {}

            for field in (
                "joint_reflex",
                "cartesian_reflex",
                "communication_constraints_violation",
                "instability_detected",
            ):

                vals = re.findall(
                    rf"^\s*{re.escape(field)}:"
                    r"\s*(true|false)",
                    state,
                    (
                        re.MULTILINE
                        | re.IGNORECASE
                    ),
                )

                bad = any(
                    value.lower() == "true"
                    for value in vals
                )

                bad_flags[field] = bad

                if bad:
                    blockers.append(
                        "ROBOT_FAULTED: "
                        f"{field}=true"
                    )

            data[
                "health_flags"
            ] = bad_flags

        # ----------------------------------------------------
        # ROS process sanity
        # ----------------------------------------------------

        for label, pattern in (
            (
                "ros2_control_node_count",
                "^/opt/ros/jazzy/lib/"
                "controller_manager/"
                "ros2_control_node ",
            ),
            (
                "robot_state_publisher_count",
                "^/opt/ros/jazzy/lib/"
                "robot_state_publisher/"
                "robot_state_publisher ",
            ),
        ):

            rc, out = self._run(
                [
                    "pgrep",
                    "-f",
                    pattern,
                ],
                timeout=2.0,
            )

            if rc in (0, 1):

                count = len(
                    [
                        line
                        for line
                        in out.splitlines()
                        if line.strip()
                    ]
                )

            else:
                count = -1

            data[label] = count

            if count != 1:
                blockers.append(
                    f"{label}={count}, expected 1"
                )

        # ----------------------------------------------------
        # Residual MoveIt process
        # ----------------------------------------------------

        rc, out = self._run(
            [
                "pgrep",
                "-f",
                "^/opt/ros/jazzy/lib/"
                "moveit_ros_move_group/"
                "move_group",
            ],
            timeout=2.0,
        )

        if rc in (0, 1):

            move_group_count = len(
                [
                    line
                    for line
                    in out.splitlines()
                    if line.strip()
                ]
            )

        else:
            move_group_count = -1

        data[
            "move_group_count"
        ] = move_group_count

        if move_group_count > 0:

            blockers.append(
                "residual move_group "
                f"process count="
                f"{move_group_count}"
            )

        return SafetyReport(
            ok=not blockers,
            blockers=blockers,
            data=data,
        )

    def real_mode_gate(
        self,
    ) -> SafetyReport:

        blockers: List[str] = []

        fr3 = self.config.hardware.get(
            "fr3",
            {},
        )

        xhand = self.config.hardware.get(
            "xhand",
            {},
        )

        if not bool(
            fr3.get(
                "real_execution_enabled",
                False,
            )
        ):
            blockers.append(
                "fr3.real_execution_enabled=false"
            )

        if not bool(
            fr3.get(
                "tracking_backend_validated",
                False,
            )
        ):
            blockers.append(
                "fr3.tracking_backend_validated=false"
            )

        if not bool(
            xhand.get(
                "combined_motion_enabled",
                False,
            )
        ):
            blockers.append(
                "xhand.combined_motion_enabled=false"
            )

        if not bool(
            xhand.get(
                "real_grasp_enabled",
                False,
            )
        ):
            blockers.append(
                "xhand.real_grasp_enabled=false"
            )

        return SafetyReport(
            ok=not blockers,
            blockers=blockers,
            data={
                "fr3_real_execution_enabled":
                    bool(
                        fr3.get(
                            "real_execution_enabled",
                            False,
                        )
                    ),

                "fr3_tracking_backend_validated":
                    bool(
                        fr3.get(
                            "tracking_backend_validated",
                            False,
                        )
                    ),

                "xhand_combined_motion_enabled":
                    bool(
                        xhand.get(
                            "combined_motion_enabled",
                            False,
                        )
                    ),

                "xhand_real_grasp_enabled":
                    bool(
                        xhand.get(
                            "real_grasp_enabled",
                            False,
                        )
                    ),
            },
        )
