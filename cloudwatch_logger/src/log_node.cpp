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

using namespace Aws::CloudWatchLogs::Utils;

LogNode::LogNode(int8_t min_log_severity, std::unordered_set<std::string> ignore_nodes) 
    : ignore_nodes_(std::move(ignore_nodes))
{
  this->log_service_ = nullptr;
  this->min_log_severity_ = min_log_severity;
}

LogNode::~LogNode() { this->log_service_ = nullptr; }

void LogNode::Initialize(const std::string & log_group, const std::string & log_stream,
                         const Aws::Client::ClientConfiguration & config, Aws::SDKOptions & sdk_options,
                         std::shared_ptr<LogServiceFactory> factory)
{
  this->log_service_ = factory->CreateLogService(log_group, log_stream, config, sdk_options);
  this->log_service_->start(); // this is where the init happens
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
      this->log_service_->batchData(FormatLogs(log_msg));
      this->log_service_->batchData(std::string("this is a test message along for the ride"));
    }
  }
}

void LogNode::TriggerLogPublisher(const ros::TimerEvent &) {
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