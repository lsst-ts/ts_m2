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


__all__ = ["Translator"]


class Translator:
    """Translator class to translate the message from component."""

    def translate(self, message: dict) -> dict:
        """Translate the message from the component to let the SAL topic to
        use.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message_payload : `dict`
            Message payload after the translation.
        """

        message_reformat = message.copy()

        message_name = message_reformat["id"]
        if message_name == "tangentForce":
            message_reformat = self._handle_tangent_force(message_reformat)
        elif message_name == "summaryState":
            message_reformat = self._handle_summary_state(message_reformat)
        elif message_name == "commandableByDDS":
            message_reformat = self._handle_commandable_by_dds(message_reformat)

        return message_reformat

    def _handle_tangent_force(self, message: dict) -> dict:
        """Handle the message of tangent force.

        The value of "lutTemperature" is [] because there is no correction of
        LUT temperature for 6 tangent links.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message : `dict`
            Reformated message.
        """

        message["lutTemperature"] = [0.0] * 6

        return message

    def _handle_summary_state(self, message: dict) -> dict:
        """Handle the message of summary state.

        Note: Reformat the summary state in controller to be the controller's
        state.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        `dict`
            Reformated message.
        """

        return dict(id="controllerState", controllerState=message["summaryState"])

    def _handle_commandable_by_dds(self, message: dict) -> dict:
        """Handle the message of commandable by data distribution system (DDS).

        TODO: Remove this function after the DM-37422 is fixed. At this moment,
        we need to connect the CSC to the ports of GUI to establish the
        TCP/IP connection.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message : `dict`
            Reformated message.
        """

        message["state"] = True

        return message
