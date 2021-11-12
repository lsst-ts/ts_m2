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
import logging
import time

from lsst.ts import salobj
from lsst.ts.utils import make_done_future

from . import TcpClient, CommandStatus, check_queue_size, MsgType


__all__ = ["Model"]


class Model:
    """Model class.

    Parameters
    ----------
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    timeout_in_second : `float`, optional
        Time limit for reading data from the TCP/IP interface (sec). (the
        default is 0.05)
    maxsize_queue : `int`, optional
        Maximum size of queue. (the default is 1000)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    client_command : `TcpClient` or None
        Command client.
    client_telemetry : `TcpClient` or None
        Telemetry client.
    queue_event : `asyncio.Queue`
        Queue of the event.
    last_command_status : `CommandStatus`
        Last command status.
    timeout : `float`
        Time limit for reading data from the TCP/IP interface (sec).
    controller_state: `lsst.ts.salobj.State`
        Controller's state.
    """

    def __init__(self, log=None, timeout_in_second=0.05, maxsize_queue=1000):

        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.client_command = None
        self.client_telemetry = None

        self.queue_event = asyncio.Queue(maxsize=int(maxsize_queue))

        self.last_command_status = CommandStatus.Unknown

        self.timeout = timeout_in_second

        self.controller_state = salobj.State.OFFLINE

        # Start the connection task or not
        self._start_connection = False

        # Task to do the connection (asyncio.Future)
        self._task_connection = make_done_future()

        # Task to analyze the message from server (asyncio.Future)
        self._task_analyze_message = make_done_future()

    def start(
        self, host, port_command, port_telemetry, sequence_generator=None, timeout=10.0
    ):
        """Start the task and connection.

        Parameters
        ----------
        host : `str`
            Host address.
        port_command : `int`
            IP port for the command server.
        port_telemetry : `int`
            IP port for the telemetry server.
        sequence_generator : `generator` or `None`, optional
            Sequence generator. (the default is None)
        timeout : `float`, optional
            Connection timeout in second. (default is 10.0)
        """

        # Instantiate the TCP/IP clients
        maxsize_queue = self.queue_event.maxsize
        self.client_command = TcpClient(
            host,
            port_command,
            timeout_in_second=self.timeout,
            log=self.log,
            sequence_generator=sequence_generator,
            maxsize_queue=maxsize_queue,
        )
        self.client_telemetry = TcpClient(
            host,
            port_telemetry,
            timeout_in_second=self.timeout,
            log=self.log,
            maxsize_queue=maxsize_queue,
        )

        # Create the tasks
        self._start_connection = True

        self._task_connection = asyncio.create_task(self._connect(timeout))
        self._task_analyze_message = asyncio.create_task(self._analyze_message())

    async def _connect(self, timeout):
        """Connect to the servers.

        Parameters
        ----------
        timeout : `float`
            Connection timeout in second.
        """

        self.log.info("Begin to connect the servers.")

        while self._start_connection:
            if self.are_clients_connected():
                await asyncio.sleep(1)
            else:
                await asyncio.gather(
                    self.client_command.connect(timeout=timeout),
                    self.client_telemetry.connect(timeout=timeout),
                )
                self.log.info("Servers are connected.")

        self.log.info("Stop the connection with servers.")

    async def _analyze_message(self):
        """Analyze the message from the command server."""

        self.log.info("Begin to analyze the message from the command server.")

        while self._start_connection:
            try:
                if not self.client_command.queue.empty():
                    message = self.client_command.queue.get_nowait()
                    self._analyze_command_status_and_event(message)
                else:
                    await asyncio.sleep(self.timeout)

            except asyncio.QueueFull:
                self.log.exception("Internal queue of event is full.")

        self.log.info("Stop the analysis of message.")

    def _analyze_command_status_and_event(self, message):
        """Analyze the command status and event.

        Parameters
        ----------
        message : `dict`
            Incoming message.
        """

        if self._is_command_status(message):
            self.last_command_status = self._get_command_status(message)
            return

        # Update the controller state
        if self._is_controller_state(message):
            self.controller_state = salobj.State(message["summaryState"])

        # Put the event message into the queue for CSC to publish
        self.queue_event.put_nowait(message)
        check_queue_size(self.queue_event, self.log)

    def _is_command_status(self, message):
        """Is the command status or not.

        Parameters
        ----------
        message : `dict`
            Incoming message.

        Returns
        -------
        bool
            True if the message is related to the command status. Else, False.
        """

        message_name = message["id"]
        if message_name in [
            CommandStatus.Success.name.lower(),
            CommandStatus.Fail.name.lower(),
            CommandStatus.Ack.name.lower(),
            CommandStatus.NoAck.name.lower(),
        ]:
            return True
        else:
            return False

    def _get_command_status(self, message):
        """Get the command status.

        Parameters
        ----------
        message : `dict`
            Incoming message.

        Returns
        -------
        `CommandStatus`
            Command status.
        """

        message_name = message["id"]
        sequence_id = message["sequence_id"]
        if sequence_id == self.client_command.last_sequence_id:
            return self._get_command_status_from_message_name(message_name)
        else:
            self.log.info(
                f"Get the command status = {message_name}. The sequence id = {sequence_id}."
            )
            return CommandStatus.Unknown

    def _get_command_status_from_message_name(self, message_name):
        """Get the command status from message name.

        Parameters
        ----------
        message_name : `str`
            Message name.

        Returns
        -------
        `CommandStatus`
            Command status.
        """

        if message_name == CommandStatus.Success.name.lower():
            return CommandStatus.Success
        elif message_name == CommandStatus.Fail.name.lower():
            return CommandStatus.Fail
        elif message_name == CommandStatus.Ack.name.lower():
            return CommandStatus.Ack
        elif message_name == CommandStatus.NoAck.name.lower():
            return CommandStatus.NoAck

    def _is_controller_state(self, message):
        """Is the controller's state or not.

        Parameters
        ----------
        message : `dict`
            Incoming message.

        Returns
        -------
        `bool`
            True if the message is the controller's state. Else, False.
        """

        return message["id"] == "summaryState"

    def are_clients_connected(self):
        """The command and telemetry sockets are connected or not.

        Returns
        -------
        bool
            True if clients are connected. Else, False.
        """
        return (
            (self.client_command is not None)
            and (self.client_telemetry is not None)
            and self.client_command.is_connected()
            and self.client_telemetry.is_connected()
        )

    async def close(self):
        """Cancel the task and close the connection.

        Note: this function is safe to call even though there is no connection.
        """

        self._start_connection = False

        self._task_connection.cancel()
        self._task_analyze_message.cancel()

        if self.client_command is not None:
            await self.client_command.close()

        if self.client_telemetry is not None:
            await self.client_telemetry.close()

        self.client_command = None
        self.client_telemetry = None

        self.last_command_status = CommandStatus.Unknown

    def assert_controller_state(self, command_name, allowed_curr_states):
        """Assert the current controller's state is allowed to do the command
        or not.

        Parameters
        ----------
        command_name : `str`
            Command name.
        allowed_curr_states : `list [lsst.ts.salobj.State]`
            Allowed current states.

        Raises
        ------
        ValueError
            When the command is not allowed in current controller's state.
        """

        # Make sure the data type of allowed_curr_states is list
        if not isinstance(allowed_curr_states, list):
            allowed_curr_states = [allowed_curr_states]

        curr_state = self.controller_state
        if curr_state not in allowed_curr_states:
            raise ValueError(
                f"{command_name} command is not allowed in controller's state {curr_state!r}."
            )

    async def clear_errors(self):
        """Clear the errors."""
        await self.write_command_to_server(
            "clearErrors", controller_state_expected=salobj.State.OFFLINE
        )

    async def write_command_to_server(
        self,
        message_name,
        message_details=None,
        timeout=10.0,
        controller_state_expected=None,
    ):
        """Write the command (message_name) to server.

        Parameters
        ----------
        message_name : `str`
            Message name to server.
        message_details : `dict` or None, optional
            Message details. (the default is None)
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        controller_state_expected : `lsst.ts.salobj.State` or None, optional
            Expected controller's state. This is only used for the commands
            related to the state transition. (the default is None)

        Raises
        ------
        OSError:
            When no TCP/IP connection.
        RuntimeError
            When the command failed.
        """

        if not self.are_clients_connected():
            raise OSError("No TCP/IP connection.")

        # Send the command
        self.last_command_status = CommandStatus.Unknown
        await self.client_command.write(
            MsgType.Command, message_name, msg_details=message_details
        )

        is_successful = await self._check_command_status(
            message_name, timeout, controller_state_expected=controller_state_expected
        )

        if not is_successful:
            raise RuntimeError(f"{message_name} command failed.")

    async def _check_command_status(
        self, command_name, timeout, controller_state_expected=None
    ):
        """Check the command status from the controller.

        Parameters
        ----------
        command_name : `str`
            Command name.
        timeout : `float`
           Timeout of command acknowledgement in second.
        controller_state_expected : `lsst.ts.salobj.State` or None, optional
            Expected controller's state. This is only used for the commands
            related to the state transition. (the default is None)

        Returns
        -------
        `bool`
            True if the command succeeds. Else, False.
        """

        # Track the command status
        time_wait_command_status_update = 0.5
        time_start = time.monotonic()
        while time.monotonic() - time_start < timeout:

            last_command_status = self.last_command_status

            if last_command_status == CommandStatus.Success:
                if controller_state_expected is None:
                    return True
                else:
                    # Wait some time to let the event loop have the time to
                    # analyze the messages
                    await asyncio.sleep(time_wait_command_status_update)

                    # Check the controller's state is expected or not
                    if self.controller_state == controller_state_expected:
                        return True

            # If false, return immediately
            elif last_command_status in (CommandStatus.Fail, CommandStatus.NoAck):
                return False

            await asyncio.sleep(time_wait_command_status_update)

        # Log the condition that the state transition is successful, but no
        # result received
        if (controller_state_expected is not None) and (
            self.controller_state == controller_state_expected
        ):
            self.log.debug(
                f"Controller's state is expected for {command_name}. But no result received."
            )
            return True

        if last_command_status == CommandStatus.Ack:
            self.log.debug(f"Only get the acknowledgement of {command_name}.")

        return False
