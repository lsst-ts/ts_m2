# This file is part of ts_m2.
#
# Developed for the Vera Rubin Observatory Telescope and Site Systems.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import asyncio

from lsst.ts.idl.enums import MTM2

from . import DetailedState, CommandStatus


__all__ = ["MockCommand"]


class MockCommand:
    """Mock command to simulate the execution of command in real hardware."""

    async def enable(self, message, model, message_event):
        """Enable the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.actuator_power_on = True
        command_success = model.switch_force_balance_system(True)

        if not command_success:
            model.actuator_power_on = False

        await message_event.write_force_balance_system_status(
            model.force_balance_system_status
        )

        # Simulate the real hardware behavior
        await asyncio.sleep(5)

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def disable(self, message, model, message_event):
        """Disable the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.switch_force_balance_system(False)
        model.actuator_power_on = False

        await message_event.write_force_balance_system_status(
            model.force_balance_system_status
        )

        return model, CommandStatus.Success

    async def standby(self, message, model, message_event):
        """Standby the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        return model, CommandStatus.Success

    async def start(self, message, model, message_event):
        """Start the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        return model, CommandStatus.Success

    async def enter_control(self, message, model, message_event):
        """Enter the control.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        return model, CommandStatus.Success

    async def exit_control(self, message, model, message_event):
        """Exit the control.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        # Sleep time is to simulate some internal inspection of
        # real system
        await asyncio.sleep(0.01)
        await message_event.write_detailed_state(DetailedState.PublishOnly)
        await asyncio.sleep(0.01)
        await message_event.write_detailed_state(DetailedState.Available)

        return model, CommandStatus.Success

    async def apply_forces(self, message, model, message_event):
        """Apply the forces in addtional to the LUT force.

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.apply_forces(message["axial"], message["tangent"])

        return model, CommandStatus.Success

    async def position_mirror(self, message, model, message_event):
        """Position the mirror.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        mirror_position_set_point = {
            "x": message["x"],
            "y": message["y"],
            "z": message["z"],
            "xRot": message["xRot"],
            "yRot": message["yRot"],
            "zRot": message["zRot"],
        }

        try:
            command_success = model.handle_position_mirror(mirror_position_set_point)
        except RuntimeError:
            command_success = False

        await message_event.write_m2_assembly_in_position(command_success)

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def reset_force_offsets(self, message, model, message_event):
        """Reset the actuator force offsets (not LUT force).

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.reset_force_offsets()

        return model, CommandStatus.Success

    async def clear_errors(self, message, model, message_event):
        """Clear the system errors.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.clear_errors()

        return model, CommandStatus.Success

    async def switch_force_balance_system(self, message, model, message_event):
        """Switch the force balance system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        command_success = model.switch_force_balance_system(message["status"])

        await message_event.write_force_balance_system_status(
            model.force_balance_system_status
        )

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def select_inclination_source(self, message, model, message_event):
        """Select the source of inclination.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        source = int(message["source"])
        model.inclination_source = MTM2.InclinationTelemetrySource(source)

        await message_event.write_inclination_telemetry_source(
            MTM2.InclinationTelemetrySource(source)
        )

        return model, CommandStatus.Success

    async def set_temperature_offset(self, message, model, message_event):
        """Set the temperature offset used in the calculation of LUT force.

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.temperature["ring"] = message["ring"]
        model.temperature["intake"] = message["intake"]
        model.temperature["exhaust"] = message["exhaust"]

        await message_event.write_temperature_offset(
            message["ring"], message["intake"], message["exhaust"]
        )

        return model, CommandStatus.Success
