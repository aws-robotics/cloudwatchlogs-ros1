^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_logger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-03-20)
------------------
* Add unit tests for cloudwatch_logger node
  - Split log_client.cpp into log_node_param_helper.cpp for
  fetching parameters, and main.cpp for the entry point
  - Setup a static library {PROJECT_NAME}_lib for accessing
  production code from the tests
  - Add tests test_log_node and test_log_node_param_helper,
  resulting in the following overall coverage rate:
  ```
  lines......: 95.5% (362 of 379 lines)
  functions..: 91.1% (92 of 101 functions)
  branches...: 34.9% (626 of 1795 branches)
  ```
  - Also correct typo in README documenting parameter name
  `min_log_severity` instead of `min_log_verbosity`
* Update to use non-legacy ParameterReader API (`#13 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/13>`_)
  * Update to use non-legacy ParameterReader API
  * increment package version
* Allow users to configure ROS output location
* Contributors: M. M, Tim Robinson, hortala
