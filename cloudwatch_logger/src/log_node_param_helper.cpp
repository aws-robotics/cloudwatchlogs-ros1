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
#include <cloudwatch_logs_common/cloudwatch_options.h>

using Aws::Client::ParameterPath;

namespace Aws {
namespace CloudWatchLogs {
namespace Utils {

Aws::AwsError ReadPublishFrequency(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  double & publish_frequency)
{
  Aws::AwsError ret =
    parameter_reader->ReadParam(ParameterPath(kNodeParamPublishFrequencyKey), publish_frequency);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      publish_frequency = kNodePublishFrequencyDefaultValue;
      AWS_LOGSTREAM_INFO(__func__,
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

Aws::AwsError ReadLogGroup(const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
                           std::string & log_group)
{
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(kNodeParamLogGroupNameKey), log_group);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      log_group = kNodeLogGroupNameDefaultValue;
      AWS_LOGSTREAM_INFO(__func__,
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

Aws::AwsError ReadLogStream(const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
                            std::string & log_stream)
{
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(kNodeParamLogStreamNameKey), log_stream);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      log_stream = kNodeLogStreamNameDefaultValue;
      AWS_LOGSTREAM_INFO(__func__,
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
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  bool & subscribe_to_rosout)
{
  Aws::AwsError ret =
    parameter_reader->ReadParam(ParameterPath(kNodeParamSubscribeToRosoutKey), subscribe_to_rosout);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      subscribe_to_rosout = kNodeSubscribeToRosoutDefaultValue;
      AWS_LOGSTREAM_INFO(
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
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  int8_t & min_log_verbosity)
{
  min_log_verbosity = kNodeMinLogVerbosityDefaultValue;

  std::string specified_verbosity;
  Aws::AwsError ret =
    parameter_reader->ReadParam(ParameterPath(kNodeParamMinLogVerbosityKey), specified_verbosity);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      AWS_LOGSTREAM_INFO(__func__, "Log verbosity configuration not found, setting to default value: "
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
        AWS_LOGSTREAM_INFO(__func__,
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

Aws::AwsError ReadPublishTopicNames(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  bool & publish_topic_names)
{
  Aws::AwsError ret =
    parameter_reader->ReadParam(ParameterPath(kNodeParamPublishTopicNamesKey), publish_topic_names);

  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      publish_topic_names = kNodePublishTopicNamesDefaultValue;
      AWS_LOGSTREAM_INFO(
      __func__,
      "Whether to publish topic names to Cloudwatch Logs configuration not found, setting to default value: "
        << kNodePublishTopicNamesDefaultValue);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(
      __func__, "Whether to publish topic names to Cloudwatch Logs is set to: " << publish_topic_names);
      break;
    default:
      publish_topic_names = kNodePublishTopicNamesDefaultValue;
      AWS_LOGSTREAM_ERROR(
        __func__,
        "Error " << ret 
        << "retrieving parameter for whether to publish topic names to Cloudwatch Logs" 
        << ", setting to default value: " << kNodePublishTopicNamesDefaultValue);
  }

  return ret;
}

Aws::AwsError ReadSubscriberList(
  const bool subscribe_to_rosout,
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  const boost::function<void(const rosgraph_msgs::Log::ConstPtr &)>& callback,
  ros::NodeHandle & nh,
  std::vector<ros::Subscriber> & subscriptions)
{
  std::vector<std::string> topics;
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(kNodeParamLogTopicsListKey), topics);

  for (const std::string& topic : topics) {
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

Aws::AwsError ReadIgnoreNodesSet(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  std::unordered_set<std::string> & ignore_nodes)
{
  std::vector<std::string> ignore_list;
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(kNodeParamIgnoreNodesKey), ignore_list);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      break;
    case Aws::AwsError::AWS_ERR_OK:
      for (const std::string & node_name : ignore_list) {
        ignore_nodes.emplace(node_name);
      }
      break;
    default:
      AWS_LOGSTREAM_ERROR(__func__, 
        "Error " << ret << " retrieving retrieving list of nodes to ignore.");
  }
  
  return ret;
}

void ReadCloudWatchOptions(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  Aws::CloudWatchLogs::CloudWatchOptions & cloudwatch_options) {

  Aws::DataFlow::UploaderOptions uploader_options{};
  Aws::FileManagement::FileManagerStrategyOptions file_manager_strategy_options;

  ReadUploaderOptions(parameter_reader, uploader_options);
  ReadFileManagerStrategyOptions(parameter_reader, file_manager_strategy_options);

  cloudwatch_options = {
    uploader_options,
    file_manager_strategy_options
  };
}

void ReadUploaderOptions(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  Aws::DataFlow::UploaderOptions & uploader_options) {

  ReadOption(
    parameter_reader,
    kNodeParamFileUploadBatchSize,
    Aws::DataFlow::kDefaultUploaderOptions.file_upload_batch_size,
    uploader_options.file_upload_batch_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamFileMaxQueueSize,
    Aws::DataFlow::kDefaultUploaderOptions.file_max_queue_size,
    uploader_options.file_max_queue_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamBatchMaxQueueSize,
    Aws::DataFlow::kDefaultUploaderOptions.batch_max_queue_size,
    uploader_options.batch_max_queue_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamBatchTriggerPublishSize,
    Aws::DataFlow::kDefaultUploaderOptions.batch_trigger_publish_size,
    uploader_options.batch_trigger_publish_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamStreamMaxQueueSize,
    Aws::DataFlow::kDefaultUploaderOptions.stream_max_queue_size,
    uploader_options.stream_max_queue_size
  );
}

void ReadFileManagerStrategyOptions(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  Aws::FileManagement::FileManagerStrategyOptions & file_manager_strategy_options) {

  ReadOption(
    parameter_reader,
    kNodeParamStorageDirectory,
    Aws::FileManagement::kDefaultFileManagerStrategyOptions.storage_directory,
    file_manager_strategy_options.storage_directory);

  ReadOption(
    parameter_reader,
    kNodeParamFilePrefix,
    Aws::FileManagement::kDefaultFileManagerStrategyOptions.file_prefix,
    file_manager_strategy_options.file_prefix);

  ReadOption(
    parameter_reader,
    kNodeParamFileExtension,
    Aws::FileManagement::kDefaultFileManagerStrategyOptions.file_extension,
    file_manager_strategy_options.file_extension);

  ReadOption(
    parameter_reader,
    kNodeParamMaximumFileSize,
    Aws::FileManagement::kDefaultFileManagerStrategyOptions.maximum_file_size_in_kb,
    file_manager_strategy_options.maximum_file_size_in_kb);

  ReadOption(
    parameter_reader,
    kNodeParamStorageLimit,
    Aws::FileManagement::kDefaultFileManagerStrategyOptions.storage_limit_in_kb,
    file_manager_strategy_options.storage_limit_in_kb);
}

void ReadOption(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  const std::string & option_key,
  const std::string & default_value,
  std::string & option_value) {
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(option_key), option_value);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      option_value = default_value;
      AWS_LOGSTREAM_INFO(__func__,
                         option_key << " parameter not found, setting to default value: " << default_value);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(__func__, option_key << " is set to: " << option_value);
      break;
    default:
      option_value = default_value;
      AWS_LOGSTREAM_ERROR(__func__,
        "Error " << ret << " retrieving option " << option_key << ", setting to default value: " << default_value);
  }
}

void ReadOption(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  const std::string & option_key,
  const size_t & default_value,
  size_t & option_value) {
  int return_value = 0;
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(option_key), return_value);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      option_value = default_value;
      AWS_LOGSTREAM_INFO(__func__,
                         option_key << " parameter not found, setting to default value: " << default_value);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      option_value = static_cast<size_t>(return_value);
      AWS_LOGSTREAM_INFO(__func__, option_key << " is set to: " << option_value);
      break;
    default:
      option_value = default_value;
      AWS_LOGSTREAM_ERROR(__func__,
        "Error " << ret << " retrieving option " << option_key << ", setting to default value: " << default_value);
  }
}

}  // namespace Utils
}  // namespace CloudWatchLogs
}  // namespace Aws
