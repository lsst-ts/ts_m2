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

from . import write_json_packet

__all__ = ["MockMessageEvent"]


class MockMessageEvent:
    """Mock message of event to simulate the message from real hardware.

    Parameters
    ----------
    writer : `asyncio.StreamWriter` or None
        Writer of the socket.

    Attributes
    ----------
    writer : `asyncio.StreamWriter` or None
        Writer of the socket.
    """

    def __init__(self, writer):

        self.writer = writer

    async def write_m2_assembly_in_position(self, in_position):
        """Write the message: M2 assembly is in position or not.

        Parameters
        ----------
        in_position : `bool`
            M2 assembly is in position or not.
        """

        msg = {"id": "m2AssemblyInPosition", "inPosition": in_position}
        await write_json_packet(self.writer, msg)

    async def write_cell_temperature_high_warning(self, hi_warning):
        """Write the message: cell temperature is high or not.

        Parameters
        ----------
        hi_warning : `bool`
            Cell temperature is high or not.
        """

        msg = {"id": "cellTemperatureHiWarning", "hiWarning": hi_warning}
        await write_json_packet(self.writer, msg)

    async def write_detailed_state(self, detailed_state):
        """Write the message: detailed state.

        Parameters
        ----------
        detailed_state : enum `DetailedState`
            M2 detailed state.
        """

        msg = {"id": "detailedState", "detailedState": int(detailed_state)}
        await write_json_packet(self.writer, msg)

    async def write_commandable_by_dds(self, state):
        """Write the message: commandable by DDS or not.

        Parameters
        ----------
        state : `bool`
            Commandable by DDS or not.
        """

        msg = {"id": "commandableByDDS", "state": state}
        await write_json_packet(self.writer, msg)

    async def write_interlock(self, state):
        """Write the message: interlock.

        Parameters
        ----------
        state : `bool`
            Interlock is on or not.
        """

        msg = {"id": "interlock", "state": state}
        await write_json_packet(self.writer, msg)

    async def write_tcp_ip_connected(self, is_connected):
        """Write the message: TCP/IP connection is on or not.

        Parameters
        ----------
        is_connected : `bool`
            TCP/IP connection is on or not.
        """

        msg = {"id": "tcpIpConnected", "isConnected": is_connected}
        await write_json_packet(self.writer, msg)

    async def write_hardpoint_list(self, actuators):
        """Write the message: hardpoint list.

        Parameters
        ----------
        actuators : `list`
            Hardpoint list.
        """

        msg = {"id": "hardpointList", "actuators": actuators}
        await write_json_packet(self.writer, msg)

    async def write_force_balance_system_status(self, status):
        """Write the message: force balance system is on or not.

        Parameters
        ----------
        status : `bool`
            Force balance system is on or not.
        """

        msg = {"id": "forceBalanceSystemStatus", "status": status}
        await write_json_packet(self.writer, msg)

    async def write_inclination_telemetry_source(self, source):
        """Write the message: inclination telemetry source.

        Parameters
        ----------
        source : enum `MTM2.InclinationTelemetrySource`
            Inclination telemetry source.
        """

        msg = {"id": "inclinationTelemetrySource", "source": int(source)}
        await write_json_packet(self.writer, msg)

    async def write_temperature_offset(self, ring, intake, exhaust):
        """Write the message: temperature offset in degree C.

        Parameters
        ----------
        ring : `list`
            Offset of ring temperatures.
        intake : `list`
            Offset of intake temperatures.
        exhaust : `list`
            Offset of exhaust temperatures.
        """

        msg = {
            "id": "temperatureOffset",
            "ring": ring,
            "intake": intake,
            "exhaust": exhaust,
        }
        await write_json_packet(self.writer, msg)
