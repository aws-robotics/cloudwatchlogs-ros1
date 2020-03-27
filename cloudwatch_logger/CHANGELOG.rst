^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_logger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.1 (2020-03-27)
------------------
* Fix linting issues found by clang-tidy 6.0 (`#56 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/56>`_)
  * clang-tidy fixes
  * clang-tidy linting issues fixed manually
* Add option to not publish topic names to Cloudwatch Logs (`#58 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/58>`_)
  * add option to not publish topic names to cloudwatch logs
  * fix unit test failure
  * add unit tests, improve code coverage
  * update documentation
  * address PR comments
  * make the deprecated constructor use constructor delegation
* Merge pull request `#51 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/51>`_ from aws-robotics/restore-tests
  Restored unit tests
* Restored unit tests
* Increment version (`#48 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/48>`_)
  * increment version for binary packaging fix
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * cleanup README
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Merge pull request `#49 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/49>`_ from aws-robotics/remove-debug-flag
  Minor updates for offline feature release
* - remove debug flag (run with level info)
  - minor README changes
* install missing library (`#47 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/47>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* increment version for offline logs feature (`#46 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/46>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Fixing std_msgs, std_srvs dependency resolution (`#44 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/44>`_)
  * Add build_depend on std_srvs, std_msgs
  * Add std_msgs to CMakeLists.txt
* Find std_srvs in CMakeLists.txt
* Offline Logs (`#42 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/42>`_)
  * Working branch with offline feature
  cr https://code.amazon.com/reviews/CR-9548516
  * Addressed review comments
  * ROS-1775: Adapt Interfaces to CW Metrics
  cr https://code.amazon.com/reviews/CR-9600347
  * ROS-2223: Add ROS Service Method to LogNode
  cr https://code.amazon.com/reviews/CR-10013824
  * Read in parameters for UploaderOptions and FileManagerOptions
  - Read in the parameters for the UploadOptions struct and the
  FileManagerOptions struct so that we can create the CloudWatchOptions
  struct from them and initialize CloudWatchLogs with it.
  cr https://code.amazon.com/reviews/CR-10144869
  * Add sample configuration options, fix bugs
  - Add sample config options to the sample_configuration.yaml file
  - Fix bugs and finish off config feature
  * Add descriptions for config options
  * Don't publish on a timer when trigger_publish_size is set.
  - When the user has specifid a batch_trigger_publish_size, don't set a
  timer to publish logs, only publish when that size limit has been hit.
  * Fixing bugs, adding docs to parameter reader
  - Fix parameter reader to give a warning if a uploader option is missing
  instead of an error.
  - Add documentation for param reader functions
  - Re-order config file options to be alphabetical, set the defaults to
  the same as the app defaults.
  - Fix output flag not working when running sample_application.launch
  cr https://code.amazon.com/reviews/CR-10331731
  * ROS-2330: [Bug] CW Logs ROS1 Timer incorrectly created
  cr https://code.amazon.com/reviews/CR-10541864
  * Move parameters to README, remove unused parameters
  - Move most of the parameters out of the sample_configuration file and
  into the README instead.
  - Make function return codes void if the return value is always OK.
  - Clean up parameter reading
  * Improve README
  * Fix un-renamed function
  * Add stream_max_queue_size parameter
  - Add reading this parameter from config file and describing it in the
  advanced parameters section of the README.
  cr https://code.amazon.com/reviews/CR-10582850
  *  - support service changes
  - fix variable names
  - update build flags
* Use standard CMake macros for adding gtest/gmock tests (`#37 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/37>`_)
  * modify cloudwatch_logger to use add_rostest_gmock()
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Update package.xml for 2.1.0 release
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* Adds a new parameter for a list of node names to ignore logs from. (`#24 <https://github.com/aws-robotics/cloudwatchlogs-ros1/issues/24>`_)
  * Adds a new parameter for a list of node names to ignore logs from.
* Removing old file that has already been refactored out into separate files.
* Contributors: AAlon, Devin Bonnie, M. M, Miaofei Mei, Nick Burek, Ryan Newell

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
