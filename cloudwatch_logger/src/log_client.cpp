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

#include <iostream>

constexpr char kNodeName[] = "cloudwatch_logger";
constexpr int kNodeSubQueueSize = 100;
constexpr char kNodeRosoutAggregatedTopicName[] = "rosout_agg";

constexpr char kNodeParamLogStreamNameKey[] = "log_stream_name";
constexpr char kNodeParamPublishFrequencyKey[] = "publish_frequency";
constexpr char kNodeParamSubscribeToRosoutKey[] = "sub_to_rosout";
constexpr char kNodeParamLogGroupNameKey[] = "log_group_name";
constexpr char kNodeParamLogTopicsListKey[] = "topics";
constexpr char kNodeParamMinLogVerbosityKey[] = "min_log_verbosity";

constexpr char kNodeLogGroupNameDefaultValue[] = "ros_log_group";
constexpr char kNodeLogStreamNameDefaultValue[] = "ros_log_stream";
constexpr int8_t kNodeMinLogVerbosityDefaultValue = rosgraph_msgs::Log::DEBUG;
constexpr double kNodePublishFrequencyDefaultValue = 5.0;
constexpr bool kNodeSubscribeToRosoutDefaultValue = true;

Aws::AwsError ReadPublishFrequency(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  double & publish_frequency)
{
  Aws::AwsError ret =
    parameter_reader->ReadDouble(kNodeParamPublishFrequencyKey, publish_frequency);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    publish_frequency = kNodePublishFrequencyDefaultValue;
    AWS_LOGSTREAM_WARN(__func__,
                       "Publish frequency configuration not found, setting to default value: "
                         << kNodePublishFrequencyDefaultValue);
  } else {
    AWS_LOGSTREAM_INFO(__func__, "Publish frequency is set to: " << publish_frequency);
  }
  return ret;
}

Aws::AwsError ReadLogGroup(std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
                           std::string & log_group)
{
  Aws::AwsError ret = parameter_reader->ReadStdString(kNodeParamLogGroupNameKey, log_group);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    log_group = kNodeLogGroupNameDefaultValue;
    AWS_LOGSTREAM_WARN(__func__,
                       "Log group name configuration not found, setting to default value: "
                         << kNodeLogGroupNameDefaultValue);
  } else {
    AWS_LOGSTREAM_INFO(__func__, "Log group name is set to: " << log_group);
  }
  return ret;
}

Aws::AwsError ReadLogStream(std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
                            std::string & log_stream)
{
  Aws::AwsError ret = parameter_reader->ReadStdString(kNodeParamLogStreamNameKey, log_stream);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    log_stream = kNodeLogStreamNameDefaultValue;
    AWS_LOGSTREAM_WARN(__func__,
                       "Log stream name configuration not found, setting to default value: "
                         << kNodeLogStreamNameDefaultValue);
  } else {
    AWS_LOGSTREAM_INFO(__func__, "Log stream name is set to: " << log_stream);
  }
  return ret;
}

Aws::AwsError ReadSubscribeToRosout(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  bool & subscribe_to_rosout)
{
  Aws::AwsError ret =
    parameter_reader->ReadBool(kNodeParamSubscribeToRosoutKey, subscribe_to_rosout);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    subscribe_to_rosout = kNodeSubscribeToRosoutDefaultValue;
    AWS_LOGSTREAM_WARN(
      __func__,
      "Whether to subscribe to rosout_agg topic configuration not found, setting to default value: "
        << kNodeSubscribeToRosoutDefaultValue);
  } else {
    AWS_LOGSTREAM_INFO(
      __func__, "Whether to subscribe to rosout_agg topic is set to: " << subscribe_to_rosout);
  }
  return ret;
}

Aws::AwsError ReadMinLogVerbosity(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  int8_t & min_log_verbosity)
{
  min_log_verbosity = kNodeMinLogVerbosityDefaultValue;

  std::string specified_verbosity;
  Aws::AwsError ret =
    parameter_reader->ReadStdString(kNodeParamMinLogVerbosityKey, specified_verbosity);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    AWS_LOGSTREAM_WARN(__func__, "Log verbosity configuration not found, setting to default value: "
                                   << kNodeMinLogVerbosityDefaultValue);
  } else {
    if ("DEBUG" == specified_verbosity) {
      min_log_verbosity = rosgraph_msgs::Log::DEBUG;
      AWS_LOG_INFO(__func__, "Log verbosity is set to DEBUG.");
    } else if ("INFO" == specified_verbosity) {
      min_log_verbosity = rosgraph_msgs::Log::INFO;
      AWS_LOG_INFO(__func__, "Log verbosity is set to INFO.");
    } else if ("WARN" == specified_verbosity) {
      min_log_verbosity = rosgraph_msgs::Log::WARN;
      AWS_LOG_INFO(__func__, "Log verbosity is set to WARN.");
    } else if ("ERROR" == specified_verbosity) {
      min_log_verbosity = rosgraph_msgs::Log::ERROR;
      AWS_LOG_INFO(__func__, "Log verbosity is set to ERROR.");
    } else if ("FATAL" == specified_verbosity) {
      min_log_verbosity = rosgraph_msgs::Log::FATAL;
      AWS_LOG_INFO(__func__, "Log verbosity is set to FATAL.");
    } else {
      AWS_LOGSTREAM_WARN(__func__,
                         "Log verbosity configuration not valid, setting to default value: "
                           << kNodeMinLogVerbosityDefaultValue);
    }
  }

  return ret;
}

Aws::AwsError ReadSubscriberList(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  std::vector<ros::Subscriber> & subscriptions, ros::NodeHandle & nh, bool & subscribe_to_rosout,
  boost::function<void(const rosgraph_msgs::Log::ConstPtr &)> callback)
{
  std::vector<std::string> topics;
  Aws::AwsError ret = parameter_reader->ReadList(kNodeParamLogTopicsListKey, topics);

  for (auto it = topics.begin(); it != topics.end(); ++it) {
    ros::Subscriber sub = nh.subscribe(*it, kNodeSubQueueSize, callback);
    AWS_LOGSTREAM_INFO(__func__, "Subscribing to topic: " << *it);
    subscriptions.push_back(sub);
  }
  if (subscribe_to_rosout) {
    ros::Subscriber sub = nh.subscribe(kNodeRosoutAggregatedTopicName, kNodeSubQueueSize, callback);
    AWS_LOG_INFO(__func__, "Subscribing to rosout_agg");
    subscriptions.push_back(sub);
  }
  return ret;
}

int main(int argc, char ** argv)
{
  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));
  ros::init(argc, argv, kNodeName);
  AWS_LOGSTREAM_INFO(__func__, "Starting " << kNodeName << "...");

  // required values
  double publish_frequency;
  std::string log_group;
  std::string log_stream;
  bool subscribe_to_rosout;
  int8_t min_log_verbosity;
  std::vector<ros::Subscriber> subscriptions;

  ros::NodeHandle nh;

  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader =
    std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  // checking configurations to set values or set to default values;
  ReadPublishFrequency(parameter_reader, publish_frequency);
  ReadLogGroup(parameter_reader, log_group);
  ReadLogStream(parameter_reader, log_stream);
  ReadSubscribeToRosout(parameter_reader, subscribe_to_rosout);
  ReadMinLogVerbosity(parameter_reader, min_log_verbosity);

  // configure aws settings
  Aws::Client::ClientConfigurationProvider client_config_provider(parameter_reader);
  Aws::Client::ClientConfiguration config = client_config_provider.GetClientConfiguration();
  Aws::SDKOptions sdk_options;

  Aws::CloudWatchLogs::Utils::LogNode cloudwatch_logger(min_log_verbosity);
  cloudwatch_logger.Initialize(log_group, log_stream, config, sdk_options);

  // callback function
  boost::function<void(const rosgraph_msgs::Log::ConstPtr &)> callback;
  callback = [&cloudwatch_logger](const rosgraph_msgs::Log::ConstPtr & log_msg) -> void {
    cloudwatch_logger.RecordLogs(log_msg);
  };

  // subscribe to additional topics, if any
  ReadSubscriberList(parameter_reader, subscriptions, nh, subscribe_to_rosout, callback);
  AWS_LOGSTREAM_INFO(__func__, "Initialized " << kNodeName << ".");

  // a ros timer that triggers log publisher to publish periodically
  ros::Timer timer =
    nh.createTimer(ros::Duration(publish_frequency),
                   &Aws::CloudWatchLogs::Utils::LogNode::TriggerLogPublisher, &cloudwatch_logger);
  ros::spin();
  AWS_LOGSTREAM_INFO(__func__, "Shutting down " << kNodeName << ".");
  Aws::Utils::Logging::ShutdownAWSLogging();
  ros::shutdown();
  return 0;
}
