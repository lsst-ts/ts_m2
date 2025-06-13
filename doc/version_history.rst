===============
Version History
===============

v0.13.16
--------

* Simplify the ``setup.py``.

v0.13.15
--------

* Restrict the range of the applied force to avoid the triggering of the force limit under the closed-loop control.

v0.13.14
--------

* Improve the ``setup.py`` to support the version of Python 3.11 and 3.12.

v0.13.13
--------

* Workaround the kafka topic from the old component.
* Adapt the modified enum of **BumpTest** in **ts_xml**.

v0.13.12
--------

* Improve the unit tests.

v0.13.11
--------

* Remove the **ts_idl**.
* Improve the unit tests.

v0.13.10
--------

* Unskip the test.

v0.13.9
-------

* Remove the workaround code.

v0.13.8
-------

* Fail the CSC if the telemetry timeouts.

v0.13.7
-------

* Fix the failed test case from ts_m2com v1.5.6.
* Refactor the state machine.

v0.13.6
-------

* Support the new command to enable/disable the temperature LUT, and use the default value from the configuration file.
* Remove the workaround code of backward compatibility of **ts_xml** v20.3.0.

v0.13.5
-------

* Fix the failed test case for the updated configuration file in **ts_config_mttcs**.

v0.13.4
-------

* Add back the ``Translator._handle_commandable_by_dds()`` before the fix of DM-45051.

v0.13.3
-------

* Add the ``M2._is_inclinometer_out_of_tma_range`` attribute and log the message when the M2 is out of the normal elevation range.

v0.13.2
-------

* Add the ``M2.is_csc_commander()``.
* Remove the ``Translator._handle_commandable_by_dds()``.

v0.13.1
-------

* Bypass the actions if the GUI is connecting to the controller.
* Fix the bug of setting the hardpoints and configuration file.

v0.13.0
-------

* Read the following settings from the **ts_config_mttcs**: inclination source, configuration file, and hardpoints.

v0.12.6
-------

* Check the current power status in ``M2.do_enable()``, ``M2.do_setHardpointList()``, and ``M2.do_setConfigurationFile()``.

v0.12.5
-------

* Update the version of ts-conda-build to 0.4 in the conda recipe.
* Use the **ILC_READ_WARNING_ERROR_CODES** from **ts_m2com**.
* Publish the disabled/bypassed ILC event.
* Remove the code related to the backward compatibily of **ts_xml** v20.1.0.
* Allow the set of hardpoints.

v0.12.4
-------

* Use the **mermaid** to replace the **PlantUML**.

v0.12.3
-------

* Report the **errorCode** event.

v0.12.2
-------

* Improve the ``M2.do_standby()``.

v0.12.1
-------

* Adapt the update in **ts_m2com**.

v0.12.0
-------

* Adapt the update in **ts_m2com**.
* Remove the ``getattr()`` to improve the performance of SAL publishment.

v0.11.3
-------

* Improve the debug message of ``M2.do_enable()``.

v0.11.2
-------

* Support the set of hardpoints in the simulation mode.
* Fix the failed test for the update of **ts_m2com**.
* Improve the tests.

v0.11.1
-------

* Add the interface to set the hardpoint list.

v0.11.0
-------

* Support the interrupt of actuator bump test and publish the status of bump test.

v0.10.0
-------

* Update the **Translator** class to avoid the ``unsigned long`` and ``unsigned long long`` values by using the ``string`` instead.
* Update the ``.ts_pre_commit_config.yaml``.

v0.9.1
------

* Remove the legacy code.
* Use the enums in **ts_xml** instead of **ts_idl**.
* Acknowledge 400 sec in ``M2.do_enable()``.

v0.9.0
-------

* Remove the ``M2.set_mount_elevation_in_position_callback()``.
* Ignore the events and telemetry that are not in **ts_xml** temporarily.
* Communicate with the cRIO directly.
* Add the enum field: ``ErrorCode.InterlockEngaged``.
* Support the **ts_xml** v19.0.0.

v0.8.2
-------

* Fix the test related to **ts_m2com** v1.0.0.

v0.8.1
-------

* Fix the test related to **ts_m2com** v0.12.0.

v0.8.0
-------

* Migrate the functions to **Controller** class in **ts_m2com**.

v0.7.11
-------

* Deal with the condition that the controller's data might not be completed.

v0.7.10
-------

* Let ``M2.do_enable()`` call ``self.cmd_enable.ack_in_progress()`` directly and remove the ``M2.begin_enable()``.

v0.7.9
------

* Adapt the **.ts_pre_commit_config.yaml**.

v0.7.8
------

* Support the mypy.

v0.7.7
------

* Adapt black v23.1.0.

v0.7.6
------

* Fix the test failure from the update of **ts_m2com** v0.9.4.

v0.7.5
------

* Increase the acknowledgement timeout in ``M2.begin_enable()`` command.
* Ignore the errors in ``M2.do_standby()`` command.

v0.7.4
------

* Use the constants from **ts_m2com**.

v0.7.3
------

* Fix the test failure from the update of **ts_m2com** v0.6.2.

v0.7.2
------

* Check the actuator forces before commanding the controller.

v0.7.1
------

* Fix the test failure from the update of **ts_m2com** v0.6.0.

v0.7.0
------

* Adapt the **ControllerCell** class in **ts_m2com** to remove the duplicated code.

v0.6.4
------

* Add the **.pre-commit-config.yaml**.
* Support the **isort**.

v0.6.3
------

* Fix the test from the update of **ts_m2com**.

v0.6.2
------

* Remove the *tests/harrisLUT*.
* Fix the test from the update of **ts_m2com**.

v0.6.1
------

* Fix the test from the update of **ts_m2com**.

v0.6.0
------

* Use the `ts_m2com <https://github.com/lsst-ts/ts_m2com>`_.
* Fix the indentation of *version_history.rst*.

v0.5.5
------

* Update the conda recipe for multiple versions of python.
* Ignore the error in `tcpip.close_stream_writer()`.

v0.5.4
------

* Build package with noarch instead of linux64 (the default).

v0.5.3
------

* Support the *pyproject.toml* file.

v0.5.2
------

* Actively monitor the connection status. If the server closes the connection, M2 CSC will detect this and transition to the **Fault** state. If there is no new telemetry for some time, there will be the warning message.

v0.5.1
------

* Update the mechanism to overwrite the connection information.

v0.5.0
------

* Update to salobj 7.
* Rename **README.rst** to **README.md** and update the related syntax.

v0.4.1
------
* In `CSC`:

  * Send ack_in_progress in `begin_` methods of state transition commands, since they are called before the state transition.
  * Send ack_in_progress for all CSC commands that use timeout information.
  * In `_telemetry_loop`, refactor how to get new messages and add information about message consumption rate. If queue is not empty, get with `get_nowait` otherwise use asynchronous method. This will cause the loop to pause and wait for new messages to arrive asynchronously, without the need to pool for new data while at the same time, reading as fast as possible when the queue is not empty. The penalty for not using empty() is about 5%.
  * In `_event_loop`, refactor how to get new messages. If queue us not empty, get with `get_nowait` otherwise use asynchronous method. This will cause the loop to pause and wait for new messages to arrive asynchronously, without the need to pool for new data while at the same time, reading as fast as possible when the queue is not empty.
  * In `do_standby`, stop loops after closing model.
  * In `close_tasks`, close model before stopping loops, or messages are still received while queue's are no longer being read.

* In `Model` class, pass `name` to the different `TcpClient` instances to allow debugging source of issues.
* Improve how `TcpClient` handles queue being filled up by adding timers for checking queue size and logging `QueueFull` exceptions. Instead of logging at every occurrence, create a timer task and only log when the timer is done. When queue is full, keep track of how many messages were lost.
* Add name attribute to `TcpClient` class to allow one to differentiate between the different instances of the class when debugging.
* In `utility.check_queue_size`, add `name` parameter for logging purposes.
* Remove usage of deprecated package `asynctest` in `test_csc`.

v0.4.0
------
* Add the **Translator** class.
* Update the **Model** class to use the **TcpClient** class.
* Update the **M2** class to use the TCP/IP interface with the updated **Model** class.
* Update the **doc/uml/m2_class.uml**.
* Move ``bin.src/run_mtm2.py`` to ``bin/run_mtm2.py``.
* Reformat the **rst** documents to follow the standard.
* Publish the document to `M2 document <https://ts-m2.lsst.io>`_.
* Depends on **ts_utils**.
* Subscribe the **MTMount** elevationInPosition event.
* Remove the **LSST_DDS_DOMAIN** in ``conda/meta.yaml``.
* Ignore the error code 0.
* Handle the special case that the **tangentForce** telemetry has no correction of LUT temeperature (empty list is used).
* Add the attribute of **controller_state** to **Model** class.
* Decouple the CSC summary state machine and controller's state machine.
* Update the ``user-guide.rst`` for the clear of error.
* Update the ``developer-guide.rst`` for the decoupling of state machines.
* Update the url of **PLANTUML_URL** in ``Jenkinsfile``.

v0.3.6
------
* Add the **MockModel** class.
* Integrate the **MockServer** with **MockModel**.

v0.3.5
------
* Add the **MockServer**, **MockMessageTelemetry**, **MockMessageEvent**, and **MockCommand** classes.
* Update the JSON packet header in **TcpClient** class.

v0.3.4
------
* Add the **TcpClient** class.
* Fix the **ts_salobj** deprecation warning of class attributes: valid_simulation_modes and version.
* Add the **config_schema.py** to fix the **ts_salobj** deprecation warning. Remove the **schema/m2.yaml**.

v0.3.3
------
* Add the **doc/version_history.rst**.
* Add the **doc/m2_class.uml**.
* Add the **Model** class.
