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

        match message_reformat["id"]:
            case "tangentForce":
                message_reformat = self._handle_tangent_force(message_reformat)
            case "config":
                message_reformat = self._handle_config(message_reformat)
            case "configurationFiles":
                message_reformat = self._handle_configuration_files(message_reformat)
            case "digitalInput":
                message_reformat = self._handle_digital_input(message_reformat)
            case "summaryFaultsStatus":
                message_reformat = self._handle_summary_faults_status(message_reformat)
            case "enabledFaultsMask":
                message_reformat = self._handle_enabled_faults_mask(message_reformat)
            case _:
                return message_reformat

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

    def _handle_config(self, message: dict) -> dict:
        """Handle the message of configuration.

        Notes
        -----
        Remove some fields that should be deleted after we rewrite the
        configuration files in ts_mtm2_cell in a latter time, or avoid these
        in the ts_m2cellcpp directly.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message : `dict`
            Reformated message.
        """

        for field in (
            "timeoutSal",
            "timeoutCrio",
            "timeoutIlc",
            "inclinometerDelta",
            "inclinometerDiffEnabled",
        ):
            message.pop(field)

        return message

    def _handle_configuration_files(self, message: dict) -> dict:
        """Handle the message of configuration files.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message : `dict`
            Reformated message.
        """

        message["files"] = ",".join(message["files"])

        return message

    def _handle_digital_input(self, message: dict) -> dict:
        """Handle the message of digital input.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message : `dict`
            Reformated message.
        """

        message["value"] = hex(message["value"])

        return message

    def _handle_summary_faults_status(self, message: dict) -> dict:
        """Handle the message of summary faults status.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message : `dict`
            Reformated message.
        """

        message["status"] = hex(message["status"])

        return message

    def _handle_enabled_faults_mask(self, message: dict) -> dict:
        """Handle the message of enabled faults mask.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        message : `dict`
            Reformated message.
        """

        message["mask"] = hex(message["mask"])

        return message
