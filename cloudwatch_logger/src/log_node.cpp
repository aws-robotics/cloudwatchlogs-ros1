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
#include <cloudwatch_logs_common/log_manager.h>
#include <cloudwatch_logs_common/log_manager_factory.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

using namespace Aws::CloudWatchLogs::Utils;

LogNode::LogNode(int8_t min_log_severity)
{
  this->log_manager_ = nullptr;
  this->min_log_severity_ = min_log_severity;
}

LogNode::~LogNode() { this->log_manager_ = nullptr; }

void LogNode::Initialize(const std::string & log_group, const std::string & log_stream,
                         Aws::Client::ClientConfiguration & config, Aws::SDKOptions & sdk_options)
{
  LogManagerFactory factory;
  this->log_manager_ = factory.CreateLogManager(log_group, log_stream, config, sdk_options);
}

void LogNode::RecordLogs(const rosgraph_msgs::Log::ConstPtr & log_msg)
{
  if (log_msg->name != "/cloudwatch_logger" && log_msg->name != "/cloudwatch_metrics_collector") {
    if (nullptr == this->log_manager_) {
      AWS_LOG_ERROR(__func__,
                    "Cannot publish CloudWatch logs with NULL CloudWatch LogManager instance.");
      return;
    }
    if (ShouldSendToCloudWatchLogs(log_msg->level)) {
      this->log_manager_->RecordLog(FormatLogs(log_msg));
    }
  }
}

void LogNode::TriggerLogPublisher(const ros::TimerEvent &) { this->log_manager_->Service(); }

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

  ss << "[topics: ";
  std::vector<std::string>::const_iterator it = log_msg->topics.begin();
  std::vector<std::string>::const_iterator end = log_msg->topics.end();
  for (; it != end; ++it) {
    const std::string & topic = *it;

    if (it != log_msg->topics.begin()) {
      ss << ", ";
    }

    ss << topic;
  }
  ss << "] " << log_msg->msg << "\n";

  return ss.str();
}