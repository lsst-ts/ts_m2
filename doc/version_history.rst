===============
Version History
===============

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
