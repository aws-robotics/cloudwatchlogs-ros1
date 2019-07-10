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

#pragma once

#include <cloudwatch_logs_common/cloudwatch_options.h>
#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <unordered_set>

namespace Aws {
namespace CloudWatchLogs {
namespace Utils {

constexpr int kNodeSubQueueSize = 100;
constexpr char kNodeRosoutAggregatedTopicName[] = "rosout_agg";

constexpr char kNodeParamLogStreamNameKey[] = "log_stream_name";
constexpr char kNodeParamPublishFrequencyKey[] = "publish_frequency";
constexpr char kNodeParamSubscribeToRosoutKey[] = "sub_to_rosout";
constexpr char kNodeParamLogGroupNameKey[] = "log_group_name";
constexpr char kNodeParamLogTopicsListKey[] = "topics";
constexpr char kNodeParamMinLogVerbosityKey[] = "min_log_verbosity";
constexpr char kNodeParamIgnoreNodesKey[] = "ignore_nodes";

/** Configuration params for Aws::DataFlow::UploaderOptions **/
constexpr char kNodeParamFileUploadBatchSize[] = "file_upload_batch_size";
constexpr char kNodeParamFileMaxQueueSize[] = "file_max_queue_size";
constexpr char kNodeParamBatchMaxQueueSize[] = "batch_max_queue_size";
constexpr char kNodeParamBatchTriggerPublishSize[] = "batch_trigger_publish_size";
constexpr char kNodeParamStreamMaxQueueSize[] = "stream_max_queue_size";

/** Configuration params for Aws::FileManagement::FileManagerStrategyOptions **/
constexpr char kNodeParamFilePrefix[] = "file_prefix";
constexpr char kNodeParamStorageDirectory[] = "storage_directory";
constexpr char kNodeParamFileExtension[] = "file_extension";
constexpr char kNodeParamMaximumFileSize[] = "maximum_file_size";
constexpr char kNodeParamStorageLimit[] = "storage_limit";

constexpr char kNodeLogGroupNameDefaultValue[] = "ros_log_group";
constexpr char kNodeLogStreamNameDefaultValue[] = "ros_log_stream";
constexpr int8_t kNodeMinLogVerbosityDefaultValue = rosgraph_msgs::Log::DEBUG;
constexpr double kNodePublishFrequencyDefaultValue = 5.0;
constexpr bool kNodeSubscribeToRosoutDefaultValue = true;

/**
 * Fetch the parameter for the log publishing frequency.
 *
 * @param parameter_reader to retrieve the parameters from.
 * @param publish_frequency the parameter is stored here when it is read successfully.
 * @return an error code that indicates whether the parameter was read successfully or not, 
 * as returned by \p parameter_reader
 */
Aws::AwsError ReadPublishFrequency(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  double & publish_frequency);

/**
 * Fetch the parameter for the AWS CloudWatch log group name.
 *
 * @param parameter_reader to retrieve the parameters from.
 * @param log_group the parameter is stored here when it is read successfully.
 * @return an error code that indicates whether the parameter was read successfully or not, 
 * as returned by \p parameter_reader
 */
Aws::AwsError ReadLogGroup(std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
                           std::string & log_group);

/**
 * Fetch the parameter for the AWS CloudWatch log stream name.
 *
 * @param parameter_reader to retrieve the parameters from.
 * @param log_stream the parameter is stored here when it is read successfully.
 * @return an error code that indicates whether the parameter was read successfully or not, 
 * as returned by \p parameter_reader
 */
Aws::AwsError ReadLogStream(std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
                            std::string & log_stream);

/**
 * Fetch the parameter for whether to subscribe to rosout_agg topic	or not.
 *
 * @param parameter_reader to retrieve the parameters from.
 * @param subscribe_to_rosout the parameter is stored here when it is read successfully.
 * @return an error code that indicates whether the parameter was read successfully or not, 
 * as returned by \p parameter_reader
 */
Aws::AwsError ReadSubscribeToRosout(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  bool & subscribe_to_rosout);

/**
 * Fetch the parameter for the minimum log severity for sending logs selectively to 
 * AWS CloudWatch Logs.
 *
 * @param parameter_reader to retrieve the parameters from.
 * @param min_log_verbosity the parameter is stored here when it is read successfully.
 * @return an error code that indicates whether the parameter was read successfully or not, 
 * as returned by \p parameter_reader
 */
Aws::AwsError ReadMinLogVerbosity(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  int8_t & min_log_verbosity);

/**
 * Fetch the parameter for the list of topics to get logs from, and subscribe \p nh
 * to them. Also subscribe to rout if \p subscribe_to_rosout is true. 
 * 
 * @param parameter_reader to retrieve the parameters from.
 * @param subscribe_to_rosout whether or not to create an additional subscription to rosout. 
 * @param callback callback to use for all new subscriptions created by this method.
 * @param nh node handler used to create the subscriptions.
 * @param subscriptions all new subscriptions created by this method are pushed here.
 * @return an error code that indicates whether the parameter was read successfully or not, 
 * as returned by \p parameter_reader
 */
Aws::AwsError ReadSubscriberList(
  bool subscribe_to_rosout,
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  boost::function<void(const rosgraph_msgs::Log::ConstPtr &)> callback,
  ros::NodeHandle & nh,
  std::vector<ros::Subscriber> & subscriptions);
  
/**
 * Fetch the set of node names to ignore incoming logs from. 
 * 
 * @param parameter_reader to retrieve the parameters from.
 * @param ignore_nodes all node names to ignore logs from are added here.
 * @return an error code that indicates whether the parameter was read successfully or not, 
 * as returned by \p parameter_reader
 */
Aws::AwsError ReadIgnoreNodesSet(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  std::unordered_set<std::string> & ignore_nodes);

/**
 * Fetch the options related to cloudwatch uploading and offline file mangement
 *
 * @param parameter_reader to retrieve the parameters from
 * @param cloudwatch_options a struct of uploader and file_manager options
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadCloudWatchOptions(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  Aws::CloudWatchLogs::CloudWatchOptions & cloudwatch_options);

/**
 * Fetch the options related to cloudwatch log uploading
 *
 * @param parameter_reader to retrieve the parameters from
 * @param uploader_options a struct of uploader options
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadUploaderOptions(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  Aws::DataFlow::UploaderOptions & uploader_options);

/**
 * Fetch the options related to cloudwatch offline file management
 *
 * @param parameter_reader to retrieve the parameters from
 * @param file_manager_strategy_options a struct of file management options
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadFileManagerStrategyOptions(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  Aws::FileManagement::FileManagerStrategyOptions & file_manager_strategy_options);

/**
 * Fetch a single string option
 *
 * @param parameter_reader to retrieve the parameters from
 * @param option_key the parameter key to read
 * @param default_value a default value if the parameter doesn't exist or is unreadble
 * @param option_value the string value for this option
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadOption(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  const std::string & option_key,
  const std::string & default_value,
  std::string & option_value);

/**
 * Fetch a single size_t option
 *
 * @param parameter_reader to retrieve the parameters from
 * @param option_key the parameter key to read
 * @param default_value a default value if the parameter doesn't exist or is unreadble
 * @param option_value the size_t value for this option
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadOption(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  const std::string & option_key,
  const size_t & default_value,
  size_t & option_value);

}  // namespace Utils
}  // namespace CloudWatchLogs
}  // namespace Aws
