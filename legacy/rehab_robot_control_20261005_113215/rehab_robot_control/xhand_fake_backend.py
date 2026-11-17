"""
Pure-software XHand state backend.

Safety:
- does NOT construct XHandControl
- does NOT access a NIC
- does NOT open real EtherCAT
- send_command() is deliberately forbidden
"""

from __future__ import annotations

from xhand_controller.xhand_control import HandState_t


class SoftwareFakeXHandBackend:
    def __init__(self):
        self.opened = False
        self.interface = None
        self.read_count = 0
        self.closed = False
        self.send_command_count = 0

    def open_ethercat(self, interface: str):
        # Required by XHandDriverCore backend contract.
        # This is ONLY an in-memory fake method.
        self.interface = str(interface)
        self.opened = True
        self.closed = False
        return True

    def list_hands_id(self):
        if not self.opened:
            return []
        return [0]

    def read_state(self, device_id: int, force_update: bool = False):
        if not self.opened:
            return False, HandState_t()

        state = HandState_t()

        for i, finger in enumerate(state.finger_state):
            finger.id = i
            finger.sensor_id = 0

            # Small deterministic software-only values.
            finger.position = 0.001 * i
            finger.torque = -20 + i
            finger.temperature = 30 + i

            finger.commboard_err = 0
            finger.jonitboard_err = 0
            finger.tipboard_err = 0

        for i, sensor in enumerate(state.sensor_data):
            sensor.calc_force.fx = i + 1
            sensor.calc_force.fy = -(i + 1)
            sensor.calc_force.fz = 10 + i
            sensor.calc_temperature = 40 + i

        self.read_count += 1
        return True, state

    def send_command(self, device_id, command):
        self.send_command_count += 1
        raise RuntimeError(
            "SoftwareFakeXHandBackend forbids send_command()"
        )

    def close_device(self):
        self.opened = False
        self.closed = True
