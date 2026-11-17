#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import sys
import time

from rehab_robot_control.xhand_readonly_session import (
    VendorXHandReadOnlyBackend,
    XHandReadOnlySession,
)


class PreflightError(RuntimeError):
    pass


def main():
    if os.geteuid() != 0:
        raise PreflightError(
            "XHand readonly preflight "
            "must run via sudo wrapper"
        )

    interface = os.environ["XHAND_IF"]

    expected_serial = os.environ.get(
        "XHAND_SERIAL",
        "",
    ).strip()

    backend = (
        VendorXHandReadOnlyBackend()
    )

    session = XHandReadOnlySession(
        backend=backend
    )

    opened = False

    try:
        device_id = session.open(
            interface
        )

        opened = True

        print(
            f"[PASS] XHand EtherCAT open "
            f"device_id={device_id}"
        )

        # ================================================
        # Serial
        # ================================================

        result = (
            backend
            ._control
            .get_serial_number(
                device_id
            )
        )

        if (
            not isinstance(result, tuple)
            or len(result) != 2
        ):
            raise PreflightError(
                "unexpected serial result"
            )

        error, serial = result

        if (
            hasattr(error, "error_code")
            and int(error.error_code) != 0
        ):
            raise PreflightError(
                "serial read failed: "
                f"{error.error_code}"
            )

        actual_serial = (
            str(serial)
            .strip()
            .strip("\x00")
        )

        print(
            f"serial={actual_serial}"
        )

        if (
            expected_serial
            and actual_serial
            != expected_serial
        ):
            raise PreflightError(
                "XHand serial mismatch"
            )

        # ================================================
        # 连续读取真实状态
        # ================================================

        valid = 0
        last_positions = None
        last_currents = None
        last_modes = None

        deadline = (
            time.monotonic()
            + 2.0
        )

        while (
            time.monotonic()
            < deadline
            and valid < 50
        ):
            sample = (
                session
                .read_fresh_state(
                    force_update=False
                )
            )

            if sample is None:
                time.sleep(0.01)
                continue

            fingers = (
                sample.state.finger_state
            )

            if len(fingers) != 12:
                raise PreflightError(
                    f"XHand joint count="
                    f"{len(fingers)}"
                )

            positions = []
            currents = []
            modes = []

            for i, finger in enumerate(
                fingers
            ):
                q = float(
                    finger.position
                )
                current = float(
                    finger.torque
                )

                if not (
                    math.isfinite(q)
                    and math.isfinite(current)
                ):
                    raise PreflightError(
                        f"joint {i} "
                        "non-finite state"
                    )

                errors = (
                    int(
                        finger.commboard_err
                    ),
                    int(
                        finger.jonitboard_err
                    ),
                    int(
                        finger.tipboard_err
                    ),
                )

                if any(errors):
                    raise PreflightError(
                        f"joint {i} "
                        f"errors={errors}"
                    )

                raw = int(
                    finger.raw_position
                )

                positions.append(q)
                currents.append(
                    current
                )
                modes.append(
                    raw & 0xF
                )

            valid += 1

            last_positions = positions
            last_currents = currents
            last_modes = modes

            time.sleep(0.01)

        if valid < 20:
            raise PreflightError(
                "insufficient valid XHand "
                f"states: {valid}"
            )

        if any(
            mode != 0
            for mode in last_modes
        ):
            raise PreflightError(
                "XHand is not fully powerless: "
                f"modes={last_modes}"
            )

        print(
            f"[PASS] valid states={valid}"
        )

        print(
            "positions =",
            [
                round(v, 6)
                for v in last_positions
            ],
        )

        print(
            "currents_mA =",
            [
                round(v, 1)
                for v in last_currents
            ],
        )

        print(
            "modes =",
            last_modes,
        )

        print(
            "[PASS] all 12 joints mode=0"
        )

        print()
        print(
            "[PASS] XHand READ-ONLY PREFLIGHT"
        )
        print(
            "No send_command() API exists "
            "in this project path."
        )

        return 0

    finally:
        if opened:
            session.close()

            print(
                "[PASS] XHand "
                "read-only session closed"
            )


if __name__ == "__main__":
    try:
        raise SystemExit(main())

    except Exception as exc:
        print(
            "\n[FAIL] "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(2)
