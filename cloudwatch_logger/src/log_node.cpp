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

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <cloudwatch_logger/log_node.h>
#include <cloudwatch_logs_common/log_batcher.h>
#include <cloudwatch_logs_common/log_service_factory.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <cloudwatch_logs_common/log_service.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>


namespace Aws {
namespace CloudWatchLogs {
namespace Utils {

LogNode::LogNode(const Options & options)
  : min_log_severity_(options.min_log_severity),
    ignore_nodes_(options.ignore_nodes),
    publish_topic_names_(options.publish_topic_names) {}

LogNode::LogNode(int8_t min_log_severity, std::unordered_set<std::string> ignore_nodes)
  : LogNode(Options(min_log_severity, true, std::move(ignore_nodes))) {}

LogNode::~LogNode() { this->log_service_ = nullptr; }

void LogNode::Initialize(const std::string & log_group, const std::string & log_stream,
                         const Aws::Client::ClientConfiguration & config, Aws::SDKOptions & sdk_options,
                         const Aws::CloudWatchLogs::CloudWatchOptions & cloudwatch_options,
                         const std::shared_ptr<LogServiceFactory>& factory)
{
  this->log_service_ = factory->CreateLogService(log_group, log_stream, config, sdk_options, cloudwatch_options);
}

bool LogNode::checkIfOnline(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {

  AWS_LOGSTREAM_DEBUG(__func__, "received request " << request);

  if (!this->log_service_) {
    response.success = false;
    response.message = "The LogService is not initialized";
    return true;
  }

  response.success = this->log_service_->isConnected();
  response.message = response.success ? "The LogService is connected" : "The LogService is not connected";

  return true;
}

bool LogNode::start() {
  bool is_started = true;
  if (this->log_service_) {
    is_started &= this->log_service_->start();
  }
  is_started &= Service::start();
  return is_started;
}

bool LogNode::shutdown() {
  bool is_shutdown = Service::shutdown();
  if (this->log_service_) {
    is_shutdown &= this->log_service_->shutdown();
  }
  return is_shutdown;
}

void LogNode::RecordLogs(const rosgraph_msgs::Log::ConstPtr & log_msg)
{
  if (0 == this->ignore_nodes_.count(log_msg->name)) {
    if (nullptr == this->log_service_) {
      AWS_LOG_ERROR(__func__,
                    "Cannot publish CloudWatch logs with NULL CloudWatch LogManager instance.");
      return;
    }
    if (ShouldSendToCloudWatchLogs(log_msg->level)) {
      auto message = FormatLogs(log_msg);
      this->log_service_->batchData(message);
    }
  }
}

void LogNode::TriggerLogPublisher(const ros::TimerEvent & /*unused*/) {
  this->log_service_->publishBatchedData();
}

bool LogNode::ShouldSendToCloudWatchLogs(const int8_t log_severity_level)
{
  return log_severity_level >= this->min_log_severity_;
}

const std::string LogNode::FormatLogs(const rosgraph_msgs::Log::ConstPtr & log_msg)
{
  std::stringstream ss;
  ss << log_msg->header.stamp << " ";

  switch (log_msg->level) {
    case rosgraph_msgs::Log::FATAL:
      ss << "FATAL ";
      break;
    case rosgraph_msgs::Log::ERROR:
      ss << "ERROR ";
      break;
    case rosgraph_msgs::Log::WARN:
      ss << "WARN ";
      break;
    case rosgraph_msgs::Log::DEBUG:
      ss << "DEBUG ";
      break;
    case rosgraph_msgs::Log::INFO:
      ss << "INFO ";
      break;
    default:
      ss << log_msg->level << " ";
  }
  ss << "[node name: " << log_msg->name << "] ";

  if (publish_topic_names_) {
    ss << "[topics: ";
    auto it = log_msg->topics.begin();
    auto end = log_msg->topics.end();
    for (; it != end; ++it) {
      const std::string & topic = *it;
      if (it != log_msg->topics.begin()) {
        ss << ", ";
      }
      ss << topic;
    }
    ss  << "] ";
  }

  ss << log_msg->msg << "\n";

  return ss.str();
}

}  // namespace Utils
}  // namespace CloudWatchLogs
}  // namespace Aws
