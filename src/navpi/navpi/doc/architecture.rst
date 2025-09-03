Architecture
============

NavPi is structured in a modular system. It uses ROS 2 components to build a modular structure that
is running on one single component.

.. drawio-image:: resources/architecture.drawio
   :align: center


Implementing things follows a couple of principles:

- All components share the same `vdb_mapping` instance that is drawn from the mapping component and
  handed to the other components using dependency injection.
- All components are implemented as lifecycle nodes.
- Potentially each component could be present multiple times where selection of the specific
  components could be handled on different levels:

  - ``PlanAndExecute`` could have a field for the global and local planner to use
  - Recovery behaviors could switch the local (and / or global) planner

Components
----------

This section should explain the single components in more detail.

NavPi Coordinator
~~~~~~~~~~~~~~~~~

The ``NavPi Coordinator`` is the central place that glues all the functionality of the pipeline
together. It basically contains a state machine that calls the individual steps given a global
``PlanAndExecute`` action goal, handles errors, calls recovery behaviors and returns the final
result.

It is not necessarily part of the ``NavPi Node`` as it might be implemented using python, e.g. using
the ``ros_bt_py`` library.

Open questions
--------------
When creating the architecture above there were a couple of open questions left:

- How should the NavPi Coordinator be implemented?
