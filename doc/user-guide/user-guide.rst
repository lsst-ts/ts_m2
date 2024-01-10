.. _User_Guide:

################
M2 User Guide
################

In automatic mode, the Telescope Control System (TCS) supervises the M2 operation.
There are 72 axial actuators and 6 tangential actuators in M2.
M2 can do the open-loop or closed-loop control in the operation.
In addition, M2 accepts the correction command of actuator forces from Main Telescope Active Optics System (MTAOS) to adjust the bending modes.
Only the axial actuators are involved in the bending mode.
There are 20 bending modes involved in this correction.
When a new correction command is received, M2 will overwrite the previous one.

M2 can do the rigid body movement in a limited range.
To do the movement, M2 should be under the closed-loop control.
It will report the new position based on the readings of 78 loading cells of actuators and the home position.
In addition, there are 6 independent measurement system (IMS) sensors to support an independent calculation of rigid body position compared with the previous one.

.. _State_Machine:

State Machine
=============

The state machine of M2 CSC follows the state machine defined in *ts_salobj* module.
When the CSC is in *STANDBY* state, there is no TCP/IP connection with the M2 controller.
The TCP/IP connection is constructed when the M2 CSC transitions to *DISABLED* state.
When the connection is on, the M2 CSC should be able to get the telemetry from the M2 controller.
You can use the SAL event ``controllerState`` to get the summary state of controller.
The default controller's state is *OFFLINE*.
When you transition the M2 CSC to *ENABLED* state, it will try to transition the M2 controller's state from *OFFLINE* to *ENABLED*.

If the M2 controller is in *FAULT* state, the M2 CSC will be put into the *FAULT* state as well.
When this happens, you should use the SAL command ``standby`` to transition the M2 CSC's state from *FAULT* to *STANDBY*.
In this process, the M2 CSC will try to clear the error in M2 controller first.
If the error is cleared, the M2 controller's state will be in *OFFLINE*.
Then, the M2 CSC will disconnect from the M2 controller and transition to *STANDBY* state.

If the M2 CSC is not in *STANDBY* state and losts the TCP/IP connection with M2 controller, it will transition to *FAULT* state and report the related error code.
You can use the SAL command ``standby`` to transtion the CSC back to *STANDBY* state.

.. _Interface:

Interface
=========

The full set of commands, events, and telemetry can be found on `MTM2 xml <https://ts-xml.lsst.io/sal_interfaces/MTM2.html>`_.
The principal use-case for this CSC is to apply the actuator forces to adjust the bending mode.
The ``applyForces`` command provides the delta accumulated force vector used in adjusting the target force settings in the closed-loop control.
The ``resetForceOffsets`` commmand causes the M2 assembly zero all force offsets in the closed-loop control.
The ``positionMirror`` command adjusts the mirror’s rigid body position based on LTS-136 relative from the home position.
The home position can be reconfigured.
The ``clearErrors`` command clear all errors in the controller.

.. _Example_Use_Case:

Example Use-Case
================

This example discusses Commanding from Python script.
Commanding from other CSCs is expected during routine operations.

After importing *ts_salobj* module, **Remote** object, a proxy for M2 CSC, is created.
The ``salobj.set_summary_state()`` call is used to switch to *ENABLED* state.

.. code:: python

    from lsst.ts import salobj
    m2 = salobj.Remote(salobj.Domain(), "MTM2")
    await m2.start_task
    await salobj.set_summary_state(m2, salobj.State.ENABLED, timeout=30)

In the *ts_salobj* v6.0.3, the user can not create the **Remote** object in ipython directly.
Instead, you need to create an event loop by yourself and do the related instantiation.
You may need to check the environment variables of **LSST_DDS_PARTITION_PREFIX** and **LSST_SITE** exist or not.
For example,

.. code:: python

    import asyncio
    from lsst.ts import salobj

    async def create_csc():
        return salobj.Remote(salobj.Domain(), "MTM2")

    loop = asyncio.get_event_loop()
    m2 = loop.run_until_complete(create_csc())

To call a M2 command, use a cmd_{nameOfCommand} property.
Such as:

.. code:: python

    await m2.cmd_clearErrors.set_start(timeout=10)

*Timeout* is specified in seconds.
The *salobj.AckTimeoutError* exception will be thrown if *timeout* seconds passed and the command is not finished.
In the simulation mode, it is safe to set the *timeout* parameter between 15 to 30 seconds.

Default values are provided for all parameters.
For example:

.. code:: python

    await m2.cmd_applyForces.set_start(timeout=10)

M2 provides methods to wait for reception of the event.
Similarly to cmd handling, use a evt_{nameOfEvent} property.
For example, to wait for the next summary state, call:

.. code:: python

    state = await m2.evt_summaryState.next(flush=False, timeout=30)

The *next* method waits for a value, returning the oldest next value (if multiple events are received).
Use *aget* to retrieve the current value (or wait for any, if the event wasn't yet received):

.. code:: python

    state = await m2.evt_summaryState.aget(timeout=30)

Telemetry is received using *tel_* prefix instead of *evt_*.

.. _Further_Reading:

Further Reading
===============

For further details, please see:

- `RemoteCommand <https://ts-salobj.lsst.io/py-api/lsst.ts.salobj.topics.RemoteCommand.html>`_
- `RemoteEvent <https://ts-salobj.lsst.io/py-api/lsst.ts.salobj.topics.RemoteEvent.html>`_
- `RemoteTelemetry <https://ts-salobj.lsst.io/py-api/lsst.ts.salobj.topics.RemoteTelemetry.html>`_
