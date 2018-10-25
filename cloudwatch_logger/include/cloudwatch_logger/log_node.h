/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <cloudwatch_logs_common/log_manager.h>
#include <cloudwatch_logs_common/log_manager_factory.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

namespace Aws {
namespace CloudWatchLogs {
namespace Utils {

class LogNode
{
public:
  /**
   * @brief Creates a new CloudWatchLogNode
   *
   * @param min_log_severity the minimum log severity level defined in the configuration file
   *                         logs with severity level equal or above get sent to CloudWatch Logs
   */
  explicit LogNode(int8_t min_log_severity);

  /**
   *  @brief Tears down a AWSCloudWatchLogNode object
   */
  ~LogNode();

  /**
   * @brief Reads creds, region, and SDK option to configure log manager
   *
   * @param log_group log group name
   * @param log_stream log stream name
   * @param config aws client configuration object
   * @param sdk_options aws sdk options
   */
  void Initialize(const std::string & log_group, const std::string & log_stream,
                  Aws::Client::ClientConfiguration & config, Aws::SDKOptions & sdk_options);

  /**
   * @brief Emits RecordLog using the log manager
   *
   * @param log_msg A log message from the subscribed topic(s)
   */
  void RecordLogs(const rosgraph_msgs::Log::ConstPtr & log_msg);

  /**
   * @brief Trigger the log manager to call its Service function to publish logs to cloudwatch
   * periodically
   *
   * @param timer A ros timer
   */
  void TriggerLogPublisher(const ros::TimerEvent &);

private:
  bool ShouldSendToCloudWatchLogs(const int8_t log_severity_level);
  const std::string FormatLogs(const rosgraph_msgs::Log::ConstPtr & log_msg);
  std::shared_ptr<Aws::CloudWatchLogs::LogManager> log_manager_;
  int8_t min_log_severity_;
};

}  // namespace Utils
}  // namespace CloudWatchLogs
}  // namespace Aws
