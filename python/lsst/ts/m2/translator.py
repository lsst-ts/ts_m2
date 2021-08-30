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

    def translate(self, message):
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

        message_reformat = self._handle_special_case(message)

        message_payload = message_reformat.copy()
        if "id" in message_payload.keys():
            message_payload.pop("id")

        return message_payload

    def _handle_special_case(self, message):
        """Handle the special case to refomat the message to match the format
        of xml.

        Note: We will begin to implement this when communication with M2 cell
        instead of M2 server.

        Parameters
        ----------
        message : `dict`
            Message from the component.

        Returns
        -------
        `dict`
            Reformated message.
        """

        return message
