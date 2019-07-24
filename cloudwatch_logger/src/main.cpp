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

using namespace Aws::CloudWatchLogs::Utils;

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
  int8_t min_log_verbosity;
  std::vector<ros::Subscriber> subscriptions;
  std::unordered_set<std::string> ignore_nodes;
  Aws::CloudWatchLogs::CloudWatchOptions cloudwatch_options;

  ros::NodeHandle nh;

  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader =
    std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  // checking configurations to set values or set to default values;
  ReadPublishFrequency(parameter_reader, publish_frequency);
  ReadLogGroup(parameter_reader, log_group);
  ReadLogStream(parameter_reader, log_stream);
  ReadSubscribeToRosout(parameter_reader, subscribe_to_rosout);
  ReadMinLogVerbosity(parameter_reader, min_log_verbosity);
  ReadIgnoreNodesSet(parameter_reader, ignore_nodes);

  ReadCloudWatchOptions(parameter_reader, cloudwatch_options);

  // configure aws settings
  Aws::Client::ClientConfigurationProvider client_config_provider(parameter_reader);
  Aws::Client::ClientConfiguration config = client_config_provider.GetClientConfiguration();

  Aws::SDKOptions sdk_options;
  sdk_options.loggingOptions.logLevel = Aws::Utils::Logging::LogLevel::Debug;

  Aws::CloudWatchLogs::Utils::LogNode cloudwatch_logger(min_log_verbosity, ignore_nodes);
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
  ReadSubscriberList(subscribe_to_rosout, parameter_reader, callback, nh, subscriptions);
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
