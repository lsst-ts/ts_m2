===============
Version History
===============

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
------

* Deal with the condition that the controller's data might not be completed.

v0.7.10
------

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
