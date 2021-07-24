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

from . import DetailedState


__all__ = ["MockCommand"]


class MockCommand:
    """Mock command to simulate the execution of command in real hardware.

    TODO (DM-30851):
    Most of code will be implemented in DM-30851.

    Attributes
    ----------
    system_enabled : `bool`
        System is in the Enabled state or not.
    """

    def __init__(self):

        # System is in the Enabled state or not
        # TODO: This attribute will be removed in the work of DM-30851
        self.system_enabled = False

    async def enable(self, message, message_event):
        """Enable the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        await message_event.write_force_balance_system_status(True)

        # Simulate the real hardware behavior
        await asyncio.sleep(5)

        self.system_enabled = True

    async def disable(self, message, message_event):
        """Disable the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        await message_event.write_force_balance_system_status(False)

        self.system_enabled = False

    async def exit_control(self, message, message_event):
        """Exit the control.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        # Sleep time is to simulate some internal inspection of
        # real system
        await asyncio.sleep(0.01)
        await message_event.write_detailed_state(DetailedState.PublishOnly)
        await asyncio.sleep(0.01)
        await message_event.write_detailed_state(DetailedState.Available)

    async def apply_forces(self, message, message_event):
        """Apply the forces in addtional to the LUT force.

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        # TODO (DM-30851):
        # Implement in DM-30851
        pass

    async def position_mirror(self, message, message_event):
        """Position the mirror.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        # TODO (DM-30851):
        # Implement in DM-30851
        pass

    async def reset_force_offsets(self, message, message_event):
        """Reset the actuator force offsets (not LUT force).

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        # TODO (DM-30851):
        # Implement in DM-30851
        pass

    async def clear_errors(self, message, message_event):
        """Clear the system errors.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        # TODO (DM-30851):
        # Implement in DM-30851
        pass

    async def switch_force_balance_system(self, message, message_event):
        """Switch the force balance system.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        await message_event.write_force_balance_system_status(message["status"])

    async def select_inclination_source(self, message, message_event):
        """Select the source of inclination.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        await message_event.write_inclination_telemetry_source(
            MTM2.InclinationTelemetrySource(message["source"])
        )

    async def set_temperature_offset(self, message, message_event):
        """Set the temperature offset used in the calculation of LUT force.

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        await message_event.write_temperature_offset(
            message["ring"], message["intake"], message["exhaust"]
        )
