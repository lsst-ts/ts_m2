.. |CSC_developer| replace:: *Tiago Ribeiro <tribeiro@lsst.org>* and *Te-Wei Tsai <ttsai@lsst.org>*
.. |CSC_product_owner| replace:: *Sandrine Thomas <sthomas@lsst.org>*

.. Note that the ts_ prefix is omitted from the title

########################
M2
########################

.. image:: https://img.shields.io/badge/GitHub-ts__m2-green.svg
    :target: https://github.com/lsst-ts/ts_m2
.. image:: https://img.shields.io/badge/Jenkins-ts__m2-green.svg
    :target: https://tssw-ci.lsst.org/job/LSST_Telescope-and-Site/job/ts_m2
.. image:: https://img.shields.io/badge/Jira-ts__m2-green.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels%20in%20(ts_m2%2C%20%20M2)
.. image:: https://img.shields.io/badge/ts__xml-MTM2-green.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/MTM2.html

.. _Overview:

Overview
========

The Main Telescope M2 Commandable SAL Component (CSC) is operating the M2 mirror control system.
M2 calculates the actuator steps (or forces) in the closed-loop control to compensate the affections from gravity and temperature based on the look-up table (LUT), loading cell of force, temeperature sensor, and inclinometer.
In addition to use the internal inclinometer, it can subscribe the elevation angle from the Main Telescope Mount (MTMount) CSC instead.
It can accept the force command from the Main Telescope Active Optics System (MTAOS) CSC to change the bending modes to reach the high image quality.
DDS/SAL (Service Abstraction Layer) is used to send or receive commands/events/telemetry among telescope's subsystems.

In automatic operation, M2 will be controlled by the Telescope Control System (TCS) and MTAOS to do the closed-loop correction and adjust the bending modes.
M2 is part of the `Main Telescope Control Packages <https://obs-controls.lsst.io/System-Architecture/Control-Packages/index.html>`_.
The backbone of CSC is using the `ts_salobj <https://ts-salobj.lsst.io>`_ library, which defines the state transitions.
The summary state machine is defined in `TCS Software Component Interface <https://docushare.lsst.org/docushare/dsweb/Get/LTS-307>`_.
The `eups <https://github.com/RobertLuptonTheGood/eups>`_ is used as the package manager.
This package also supports the `conda <https://docs.conda.io/en/latest>`_ package manager.

The badges above navigate to the GitHub repository for the CSC code, Jenkins CI jobs, Jira issues, and communication interface for the software.

.. _User_Documentation:

User Documentation
==================

Observatory operators and other interested parties should consult the user guide for insights into M2 operations.

.. toctree::
    user-guide/user-guide
    :maxdepth: 1

.. _Configuration:

Configuring the M2
=====================

M2's configuration is described at the following link.

.. toctree::
    configuration/configuration
    :maxdepth: 1

.. _Development_Documentation:

Development Documentation
=========================

Classes and their methods, and how to get involved in the M2 development is described in this section.

.. toctree::
    developer-guide/developer-guide
    :maxdepth: 1

.. _Version_History:

Version History
===============

The version history is at the following link.

.. toctree::
    version_history
    :maxdepth: 1

The released version is `here <https://github.com/lsst-ts/ts_m2/releases>`_.

.. _Contact_Personnel:

Contact Personnel
=================

For questions not covered in the documentation, emails should be addressed to the developers: |CSC_developer|.
The product owner is |CSC_product_owner|.

This page was last modified |today|.
