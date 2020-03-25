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
#include <cloudwatch_logger/log_node.h>
#include <cloudwatch_logger/log_node_param_helper.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <iostream>
#include <unordered_set>
#include <string>

#include <cloudwatch_logs_common/cloudwatch_options.h>

constexpr char kNodeName[] = "cloudwatch_logger";

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
  Aws::SDKOptions sdk_options;
  Aws::CloudWatchLogs::Utils::LogNode::Options logger_options;
  Aws::CloudWatchLogs::CloudWatchOptions cloudwatch_options;

  ros::NodeHandle nh;

  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader =
    std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  // checking configurations to set values or set to default values;
  Aws::CloudWatchLogs::Utils::ReadPublishFrequency(parameter_reader, publish_frequency);
  Aws::CloudWatchLogs::Utils::ReadLogGroup(parameter_reader, log_group);
  Aws::CloudWatchLogs::Utils::ReadLogStream(parameter_reader, log_stream);
  Aws::CloudWatchLogs::Utils::ReadSubscribeToRosout(parameter_reader, subscribe_to_rosout);
  Aws::CloudWatchLogs::Utils::ReadMinLogVerbosity(parameter_reader, logger_options.min_log_severity);
  Aws::CloudWatchLogs::Utils::ReadPublishTopicNames(parameter_reader, logger_options.publish_topic_names);
  Aws::CloudWatchLogs::Utils::ReadIgnoreNodesSet(parameter_reader, logger_options.ignore_nodes);
  Aws::CloudWatchLogs::Utils::ReadCloudWatchOptions(parameter_reader, cloudwatch_options);

  // configure aws settings
  Aws::Client::ClientConfigurationProvider client_config_provider(parameter_reader);
  Aws::Client::ClientConfiguration config = client_config_provider.GetClientConfiguration();

  Aws::CloudWatchLogs::Utils::LogNode cloudwatch_logger(logger_options);
  cloudwatch_logger.Initialize(log_group, log_stream, config, sdk_options, cloudwatch_options);

  ros::ServiceServer service = nh.advertiseService(kNodeName,
                                                   &Aws::CloudWatchLogs::Utils::LogNode::checkIfOnline,
                                                   &cloudwatch_logger);

  cloudwatch_logger.start();

  // callback function
  boost::function<void(const rosgraph_msgs::Log::ConstPtr &)> callback;
  callback = [&cloudwatch_logger](const rosgraph_msgs::Log::ConstPtr & log_msg) -> void {
    cloudwatch_logger.RecordLogs(log_msg);
  };

  // subscribe to additional topics, if any
  std::vector<ros::Subscriber> subscriptions;
  Aws::CloudWatchLogs::Utils::ReadSubscriberList(subscribe_to_rosout, parameter_reader, callback, nh, subscriptions);
  AWS_LOGSTREAM_INFO(__func__, "Initialized " << kNodeName << ".");

  bool publish_when_size_reached = cloudwatch_options.uploader_options.batch_trigger_publish_size
    != Aws::DataFlow::kDefaultUploaderOptions.batch_trigger_publish_size;

  ros::Timer timer;
  // Publish on a timer if we are not publishing on a size limit.
  if (!publish_when_size_reached) {
    timer =
      nh.createTimer(ros::Duration(publish_frequency),
                     &Aws::CloudWatchLogs::Utils::LogNode::TriggerLogPublisher,
                     &cloudwatch_logger);
  }

  ros::spin();

  AWS_LOGSTREAM_INFO(__func__, "Shutting down " << kNodeName << ".");
  cloudwatch_logger.shutdown();
  Aws::Utils::Logging::ShutdownAWSLogging();
  ros::shutdown();

  return 0;
}
