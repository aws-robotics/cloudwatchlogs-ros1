# cloudwatch_logger


## Overview
The **`cloudwatch_logger`** node enables logs generated in a ROS system to get sent to AWS CloudWatch Logs. Out of the box, this node provides the ability to subscribe to the **`/rosout_agg`**  (rosout aggregated) topic, which all logs using the ROS logging framework will be published to, and sends logs to the AWS CloudWatch Logs service. Logs can be sent to AWS CloudWatch Logs selectively based on log severity. The **`cloudwatch_logger`** node can also subscribe to other topics if logs are not sent to **`/rosout_agg`**, and it is able to unsubscribe to the rosout_agg topic for getting logs.

The **`cloudwatch_logger`** node wraps the [aws-sdk-c++] in a ROS service API.

**Amazon CloudWatch Logs Summary**: AWS CloudWatch Logs can monitor applications and systems using log data. You can create alarms in CloudWatch and receive notifications of particular API activity as captured by CloudTrail and use the notification to perform troubleshooting. By default, logs are kept indefinitely and never expire. You can adjust the retention policy for each log group, keeping the indefinite retention, or choosing a retention periods between 10 years and one day. AWS CloudWatch Logs stores your log data in highly durable storage.

**Features in Active Development**:
- Offline caching: now, a batch of logs will be dropped if the node fails to send them to AWS CloudWatch Logs. We will enable offline caching and try to save logs.
- Send logs to different log groups/streams: now, one node will send logs to one log stream within one log group. We will enable a node to send logs to different log groups/streams.

**Keywords**: ROS Application logs, System logs, AWS CloudWatch Logs service

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS Distributions
- Kinetic
- Melodic

### Build status
* Travis CI:
    * "master" branch [![Build Status](https://travis-ci.org/aws-robotics/cloudwatchlogs-ros1.svg?branch=master)](https://travis-ci.org/aws-robotics/cloudwatchlogs-ros1/branches)
    * "release-latest" branch [![Build Status](https://travis-ci.org/aws-robotics/cloudwatchlogs-ros1.svg?branch=release-latest)](https://travis-ci.org/aws-robotics/cloudwatchlogs-ros1/branches)
* ROS build farm:
    * ROS Kinetic @ u16.04 Xenial [![Build Status](http://build.ros.org/job/Kbin_uX64__cloudwatch_logger__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__cloudwatch_logger__ubuntu_xenial_amd64__binary)
    * ROS Melodic @ u18.04 Bionic [![Build Status](http://build.ros.org/job/Mbin_uB64__cloudwatch_logger__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__cloudwatch_logger__ubuntu_bionic_amd64__binary)


## Installation

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files] helpful.

This node will require the following AWS account IAM role permissions:
- `logs:PutLogEvents`
- `logs:DescribeLogStreams`
- `logs:CreateLogStream`
- `logs:CreateLogGroup`

### Binaries
On Ubuntu you can install the latest version of this package using the following command

        sudo apt-get update
        sudo apt-get install -y ros-$ROS_DISTRO-cloudwatch-logger

### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development. 

- Create a ROS workspace and a source directory

        mkdir -p ~/ros-workspace/src

- Clone the package into the source directory . 

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/cloudwatchlogs-ros1.git -b release-latest

- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
_Note: If building the master branch instead of a release branch you may need to also checkout and build the master branches of the packages this package depends on._

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/setup.bash

- Build and run the unit tests

        colcon test --packages-select cloudwatch_logs_common && colcon test-result --all


## Launch Files
An example launch file called `sample_application.launch` is provided.

## Usage

### Run the node
- **With** launch file using parameters in .yaml format (example provided)
  - ROS: `roslaunch cloudwatch_logger sample_application.launch`

- **Without** launch file using default values
  - ROS: `rosrun cloudwatch_logger cloudwatch_logger`

### Send a test log message
- `rostopic pub -1 /rosout rosgraph_msgs/Log '{header: auto, level: 2, name: test_log, msg: test_cloudwatch_logger, function: test_logs, line: 1}'`

### Verify that the test log message was successfully sent to CloudWatch Logs
- Go to your AWS account
- Find CloudWatch and click into CloudWatch
- On the upper right corner, change region to `Oregon` if you launched the node using the launch file, or change to `N. Virginia` if you launched the node without using the launch file
- Select `Logs` from the left-hand side menu
- **With** launch file: The name of the log group should be `robot_application_name` and the log stream should be `device name`
- **Without** launch file: The name of the log group should be `ros_log_group` and the log stream should be `ros_log_stream`


## Configuration File and Parameters
An example configuration file called `sample_configuration.yaml` is provided. When the parameters are absent in the ROS parameter server, default values are used, thus all parameters are optional. See table below for details.

| Parameter Name | Description | Type | Allowed Values | Default |
| -------------- | ----------- | ---- | -------------- | ------------ |
| sub_to_rosout  | Whether to subscribe to *rosout_agg* topic | *bool* | true/false | true |
| publish_frequency | Log publishing frequency in seconds | *double* | number | 5.0 |
| log_group_name | AWS CloudWatch log group name | *std::string* | 'string'<br/>*note*: Log group names must be unique within a region foran AWS account | ros_log_group |
| log_stream_name | AWS CloudWatch log stream name | *std::string* | 'string'<br/>*note*: The : (colon) and * (asterisk) characters are not allowed | ros_log_stream |
| topics | A list of topics to get logs from (excluding `rosout_agg`) | *std::vector<std::string>* | ['string', 'string', 'string'] | `[]` |
| min_log_verbosity | The minimum log severity for sending logs selectively to AWS CloudWatch Logs, log messages with a severity lower than `min_log_verbosity` will be ignored | *std::string* | DEBUG/INFO/WARN/ERROR/FATAL | DEBUG |
| publish_topic_names | Whether or not to include topic name information in the log messsages that are uploaded to AWS CloudWatch Logs | *bool* | true/false | true |
| storage_directory | The location where all offline metrics will be stored | *string* | string | ~/.ros/cwlogs/ |
| storage_limit | The maximum size of all offline storage files in KB. Once this limit is reached offline logs will start to be deleted oldest first. | *int* | number | 1048576 |
| aws_client_configuration | AWS region configuration | *std::string* | *region*: "us-west-2"/"us-east-1"/"us-east-2"/etc. | region: us-west-2 |

### Advanced Configuration Parameters
Most users won't need to touch these parameters, they are useful if you want fine grained control over how your logs are stored offline and uploaded to CloudWatch. 

| Parameter Name | Description | Type | Default |
| ------------- | -----------------------------------------------------------| ------------- | ------------ |
| batch_max_queue_size | The maximum number logs to add to the CloudWatch upload queue before they start to be written to disk | *int* | 1024 |
| batch_trigger_publish_size | Only publish logs to CloudWatch when there are this many items in the queue. When this is set the publishing of logs on a constant timer is disabled. This must be smaller than batch_max_queue_size. Logs uploaded from offline storage are not affected by this. | *int* | *unset* |
| file_max_queue_size | The max number of batches in the queue, each of size file_upload_batch_size, when reading and uploading from offline storage files | *int* | 5 |
| file_upload_batch_size | The size of each batch of logs in the queue, when reading and uploading from offline storage files | *int* | 50 |
| file_prefix | A prefix to add to each offline storage file so they're easier to identify later | *string* | cwlog |
| file_extension | The extension for all offline storage files | *string* | .log |
| maximum_file_size | The maximum size each offline storage file in KB | *int* | 1024 |
| stream_max_queue_size | The maximum number of batches in the queue to stream to CloudWatch. If this queue is full subsequent batches of logs will be written to disk. | *int* | 3 |

## Node

### cloudwatch_logger
Send logs in a ROS system to AWS CloudWatch Logs service.

#### Subscribed Topics
- **`/rosout_agg`**

  By default in ROS, all logs from applications using ROS standard logging framework are sent to *rosout_agg* topic. The *cloudwatch_logger* subscribes to *rosout_agg* by default.

- **`/other_topics`**

  The *cloudwatch_logger* subscribes to other topics if the parameter server has the topic names in *cloudwatch_logger* namespace.

#### Published Topics
None

#### Services
None


## Bugs & Feature Requests
Please contact the team directly if you would like to request a feature.

Please report bugs in [Issue Tracker].


[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/
[AWS Configuration and Credential Files]: https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html
[aws-sdk-c++]: https://github.com/aws/aws-sdk-cpp
[Issue Tracker]: https://github.com/aws-robotics/cloudwatchlogs-ros1/issues
[ROS]: http://www.ros.org
