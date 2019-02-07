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

#include <cloudwatch_logger/log_node_param_helper.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <aws/core/utils/logging/LogMacros.h>

using namespace Aws::Client;

namespace Aws {
namespace CloudWatchLogs {
namespace Utils {

Aws::AwsError ReadPublishFrequency(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  double & publish_frequency)
{
  Aws::AwsError ret =
    parameter_reader->ReadParam(ParameterPath(kNodeParamPublishFrequencyKey), publish_frequency);
  switch (ret)
  {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      publish_frequency = kNodePublishFrequencyDefaultValue;
      AWS_LOGSTREAM_WARN(__func__,
                         "Publish frequency configuration not found, setting to default value: "
                         << kNodePublishFrequencyDefaultValue);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(__func__, "Publish frequency is set to: " << publish_frequency);
      break;
    default:
      publish_frequency = kNodePublishFrequencyDefaultValue;
      AWS_LOGSTREAM_ERROR(__func__,
                         "Error " << ret << " retrieving publish frequency, setting to default value: "
                         << kNodePublishFrequencyDefaultValue);
    
  }
  return ret;
}

Aws::AwsError ReadLogGroup(std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
                           std::string & log_group)
{
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(kNodeParamLogGroupNameKey), log_group);
  switch (ret)
  {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      log_group = kNodeLogGroupNameDefaultValue;
      AWS_LOGSTREAM_WARN(__func__,
                       "Log group name configuration not found, setting to default value: "
                         << kNodeLogGroupNameDefaultValue);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(__func__, "Log group name is set to: " << log_group);
      break;
    default:
      log_group = kNodeLogGroupNameDefaultValue;
      AWS_LOGSTREAM_ERROR(__func__,
                         "Error " << ret << "retrieving log group name configuration, setting to default value: "
                         << kNodeLogGroupNameDefaultValue);
  }
  return ret;
}

Aws::AwsError ReadLogStream(std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
                            std::string & log_stream)
{
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(kNodeParamLogStreamNameKey), log_stream);
  switch (ret)
  {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      log_stream = kNodeLogStreamNameDefaultValue;
      AWS_LOGSTREAM_WARN(__func__,
                         "Log stream name configuration not found, setting to default value: "
                         << kNodeLogStreamNameDefaultValue);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(__func__, "Log stream name is set to: " << log_stream);
      break;
    default:
      log_stream = kNodeLogStreamNameDefaultValue;
      AWS_LOGSTREAM_ERROR(__func__,
                         "Error " << ret << "retrieving log stream name configuration, setting to default value: "
                         << kNodeLogStreamNameDefaultValue);
  }
  return ret;
}

Aws::AwsError ReadSubscribeToRosout(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  bool & subscribe_to_rosout)
{
  Aws::AwsError ret =
    parameter_reader->ReadParam(ParameterPath(kNodeParamSubscribeToRosoutKey), subscribe_to_rosout);
  switch (ret)
  {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      subscribe_to_rosout = kNodeSubscribeToRosoutDefaultValue;
      AWS_LOGSTREAM_WARN(
      __func__,
      "Whether to subscribe to rosout_agg topic configuration not found, setting to default value: "
        << kNodeSubscribeToRosoutDefaultValue);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(
      __func__, "Whether to subscribe to rosout_agg topic is set to: " << subscribe_to_rosout);
      break;
    default:
      subscribe_to_rosout = kNodeSubscribeToRosoutDefaultValue;
      AWS_LOGSTREAM_ERROR(
        __func__,
        "Error " << ret 
        << "retrieving parameter for whether to subscribe to rosout_agg topic configuration " 
        << ", setting to default value: " << kNodeSubscribeToRosoutDefaultValue);
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
    parameter_reader->ReadParam(ParameterPath(kNodeParamMinLogVerbosityKey), specified_verbosity);
  switch (ret)
  {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      AWS_LOGSTREAM_WARN(__func__, "Log verbosity configuration not found, setting to default value: "
                                   << kNodeMinLogVerbosityDefaultValue);
      break;
    case Aws::AwsError::AWS_ERR_OK:
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
        ret = AwsError::AWS_ERR_PARAM;
        AWS_LOGSTREAM_WARN(__func__,
                          "Log verbosity configuration not valid, setting to default value: "
                           << kNodeMinLogVerbosityDefaultValue);
      }
      break;
    default:
      AWS_LOGSTREAM_ERROR(__func__, 
        "Error " << ret << " retrieving log verbosity configuration, setting to default value: "
        << kNodeMinLogVerbosityDefaultValue);
  }
  return ret;
}

Aws::AwsError ReadSubscriberList(
  const bool subscribe_to_rosout,
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  boost::function<void(const rosgraph_msgs::Log::ConstPtr &)> callback,
  ros::NodeHandle & nh,
  std::vector<ros::Subscriber> & subscriptions)
{
  std::vector<std::string> topics;
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(kNodeParamLogTopicsListKey), topics);

  for (const std::string& topic : topics)
  {
    ros::Subscriber sub = nh.subscribe(topic, kNodeSubQueueSize, callback);
    AWS_LOGSTREAM_INFO(__func__, "Subscribing to topic: " << topic);
    subscriptions.push_back(sub);
  }
  if (subscribe_to_rosout) {
    ros::Subscriber sub = nh.subscribe(kNodeRosoutAggregatedTopicName, kNodeSubQueueSize, callback);
    AWS_LOG_INFO(__func__, "Subscribing to rosout_agg");
    subscriptions.push_back(sub);
  }
  return ret;
}


}  // namespace Utils
}  // namespace CloudWatchLogs
}  // namespace Aws
