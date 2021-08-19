===============
Version History
===============

v0.4.0
------

* Update the **Model** class to use the **TcpClient** class.
* Update the **M2** class to use the TCP/IP interface with the updated **Model** class.
* Update the **doc/uml/m2_class.uml**.
* Move ``bin.src/run_mtm2.py`` to ``bin/run_mtm2.py``.

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
* Add the **config_schema.py** to fix the **ts_salobj** deprecation warning.
Remove the **schema/m2.yaml**.

v0.3.3
------

* Add the **doc/version_history.rst**.
* Add the **doc/m2_class.uml**.
* Add the **Model** class.
