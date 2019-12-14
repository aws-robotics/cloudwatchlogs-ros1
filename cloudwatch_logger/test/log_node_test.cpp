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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cloudwatch_logger/log_node.h>
#include <cloudwatch_logs_common/log_service.h>
#include <cloudwatch_logs_common/log_batcher.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <cloudwatch_logs_common/log_service_factory.h>
#include <rosgraph_msgs/Log.h>

#include <utility>

using namespace Aws::CloudWatchLogs;
using namespace Aws::CloudWatchLogs::Utils;
using namespace Aws::FileManagement;
using ::testing::_;
using ::testing::AllOf;
using ::testing::HasSubstr;
using ::testing::Return;
using ::testing::StrEq;
using ::testing::Eq;
using ::testing::InSequence;

class LogServiceFactoryMock : public LogServiceFactory
{
  public:
    MOCK_METHOD5(CreateLogService,
      std::shared_ptr<LogService>(
        const std::string & log_group,
        const std::string & log_stream,
        const Aws::Client::ClientConfiguration & client_config,
        const Aws::SDKOptions & sdk_options,
        const CloudWatchOptions & cloudwatch_option
      )
    );
};

class LogBatcherMock : public LogBatcher
{
public:

  MOCK_METHOD0(PublishBatchedData, bool());
};

class LogPublisherMock : public LogPublisher
{
public:
    LogPublisherMock(const std::string & log_group,
                     const std::string & log_stream,
                     const Aws::Client::ClientConfiguration & client_config)
                      : LogPublisher(log_group, log_stream, client_config) {}
};

class LogServiceMock : public LogService
{
public:
    LogServiceMock(std::shared_ptr<Publisher<LogCollection>> log_publisher,
               std::shared_ptr<DataBatcher<LogType>> log_batcher,
               std::shared_ptr<FileUploadStreamer<LogCollection>> log_file_upload_streamer = nullptr)
               : LogService(std::move(log_publisher), std::move(log_batcher), std::move(log_file_upload_streamer)) {}

    MOCK_METHOD1(batchData, bool(const std::string & data_to_batch));
    MOCK_METHOD0(start, bool());
    MOCK_METHOD0(shutdown, bool());
    MOCK_METHOD0(publishBatchedData, bool());
};

class LogNodeFixture : public ::testing::Test
{
protected:

  std::shared_ptr<LogServiceMock> log_service_;
  std::shared_ptr<LogServiceFactoryMock> log_service_factory_;
  std::shared_ptr<LogBatcherMock> log_batcher_;
  std::shared_ptr<LogPublisherMock> log_publisher_;
  std::shared_ptr<LogNode> log_node_;

  rosgraph_msgs::Log log_message_;

  void SetUp() override
  {
    log_message_.name = "NjhkYjRkZjQ3N2Qw";
    log_message_.msg = "ZjQxOGE2MWM5MTFkMWNjMDVkMGY2OTZm";
    log_message_.level = rosgraph_msgs::Log::DEBUG;
    Aws::Client::ClientConfiguration config;

    log_publisher_ = std::make_shared<LogPublisherMock>(
            std::string("test_group"), std::string("test_stream"), config);
    log_batcher_ = std::make_shared<LogBatcherMock>();
    log_service_ = std::make_shared<LogServiceMock>(log_publisher_, log_batcher_);
    log_service_factory_ = std::make_shared<LogServiceFactoryMock>();
  }

  std::shared_ptr<LogNode> BuildTestSubject(int8_t severity_level = rosgraph_msgs::Log::DEBUG,
      std::unordered_set<std::string> ignore_nodes = std::unordered_set<std::string>())
  {
    return std::make_shared<LogNode>(severity_level, ignore_nodes);
  }

  rosgraph_msgs::Log::ConstPtr MessageToConstptr(rosgraph_msgs::Log log_message)
  {
    return boost::make_shared<rosgraph_msgs::Log const>(log_message);
  }

  void InitializeLogNode(std::shared_ptr<LogNode> & ln) {

    log_node_ = ln;

    std::string log_group = "YjFjMTM5YTEyNzliMjdlNjBlZGY1ZjQy";
    std::string log_stream = "hZDExYmNmMGJkMjMw";
    Aws::Client::ClientConfiguration config;
    Aws::SDKOptions sdk_options;
    Aws::CloudWatchLogs::CloudWatchOptions cloudwatch_options;

    EXPECT_CALL(*log_service_factory_,
        CreateLogService(StrEq(log_group), StrEq(log_stream), Eq(config), _, _))
      .WillOnce(Return(log_service_));

    log_node_->Initialize(log_group, log_stream, config, sdk_options, cloudwatch_options, log_service_factory_);

    EXPECT_CALL(*log_service_, start()).Times(1);

    log_node_->start();
  }

  void TearDown() override {
    if(log_node_) {
      EXPECT_CALL(*log_service_, shutdown()).Times(1);
      log_node_->shutdown();
    }
  }
};

TEST_F(LogNodeFixture, TestInitialize)
{
  std::shared_ptr<LogNode> test_subject = BuildTestSubject();
  InitializeLogNode(test_subject);
}

TEST_F(LogNodeFixture, TestRecordLogsUninitialized)
{
  std::shared_ptr<LogNode> test_subject = BuildTestSubject();

  EXPECT_CALL(*log_service_, batchData(_)).Times(0);
  EXPECT_CALL(*log_service_, start()).Times(0);

  test_subject->RecordLogs(MessageToConstptr(log_message_));
}

TEST_F(LogNodeFixture, TestRecordLogsInitialized)
{
  std::shared_ptr<LogNode> test_subject = BuildTestSubject();

  EXPECT_CALL(*log_service_, batchData(_)).Times(1);

  InitializeLogNode(test_subject);
  test_subject->RecordLogs(MessageToConstptr(log_message_));
}

TEST_F(LogNodeFixture, TestRecordLogSevBelowMinSeverity)
{
  std::shared_ptr<LogNode> test_subject = BuildTestSubject(rosgraph_msgs::Log::ERROR);

  InitializeLogNode(test_subject);

  ON_CALL(*log_service_, batchData(_))
  .WillByDefault(Return(true));
  EXPECT_CALL(*log_service_, batchData(_))
  .Times(0);

  log_message_.level = rosgraph_msgs::Log::DEBUG;
  test_subject->RecordLogs(MessageToConstptr(log_message_));
  log_message_.level = rosgraph_msgs::Log::INFO;
  test_subject->RecordLogs(MessageToConstptr(log_message_));
  log_message_.level = rosgraph_msgs::Log::WARN;
  test_subject->RecordLogs(MessageToConstptr(log_message_));
}

TEST_F(LogNodeFixture, TestRecordLogSevEqGtMinSeverity)
{
  std::shared_ptr<LogNode> test_subject = BuildTestSubject(rosgraph_msgs::Log::ERROR);

  InitializeLogNode(test_subject);

  ON_CALL(*log_service_, batchData(_))
    .WillByDefault(Return(true));
  EXPECT_CALL(*log_service_, batchData(_))
    .Times(2);

  log_message_.level = rosgraph_msgs::Log::ERROR;
  test_subject->RecordLogs(MessageToConstptr(log_message_));
  log_message_.level = rosgraph_msgs::Log::FATAL;
  test_subject->RecordLogs(MessageToConstptr(log_message_));

  ON_CALL(*log_service_, publishBatchedData())
  .WillByDefault(Return(true));
  EXPECT_CALL(*log_service_, publishBatchedData())
  .Times(1);

  // this is usually called by a timer, manually test
  ros::TimerEvent timer_event;
  test_subject->TriggerLogPublisher(timer_event);
}

TEST_F(LogNodeFixture, TestRecordLogTopicsOk)
{
  std::shared_ptr<LogNode> test_subject = BuildTestSubject(rosgraph_msgs::Log::DEBUG);

  InitializeLogNode(test_subject);

  const char* node_name2 = "zNzQwNjU4NWRi";
  std::ostringstream log_name_reference_stream2;
  log_name_reference_stream2 << "[node name: " << node_name2 << "]";

  const char* topic1 = "ZjlkYmUzOTI5ODA0ZT";
  const char* topic2 = "jNjIxMWRm";
  std::ostringstream log_topics_reference_stream2;
  log_topics_reference_stream2 << "[topics: " << topic1 << ", " << topic2 << "] ";

  {
    InSequence record_log_seq;

    std::ostringstream log_name_reference_stream1;
    log_name_reference_stream1 << "[node name: " << log_message_.name << "]";
    std::ostringstream log_topics_reference_stream1;
    log_topics_reference_stream1 << "[topics: ]";

    EXPECT_CALL(*log_service_,
      batchData(AllOf(
        HasSubstr("DEBUG"), HasSubstr(log_message_.msg),
        HasSubstr(log_name_reference_stream1.str()), HasSubstr(log_topics_reference_stream1.str())
        )))
      .WillOnce(Return(true));

    EXPECT_CALL(*log_service_,
            batchData(AllOf(
        HasSubstr("INFO"), HasSubstr(log_message_.msg),
        HasSubstr(log_name_reference_stream2.str()), HasSubstr(log_topics_reference_stream1.str())
        )))
      .WillOnce(Return(true));

    EXPECT_CALL(*log_service_,
            batchData(AllOf(
        HasSubstr("WARN"), HasSubstr(log_message_.msg),
        HasSubstr(log_name_reference_stream1.str()), HasSubstr(log_topics_reference_stream2.str())
        )))
      .WillOnce(Return(true));
  }

  log_message_.level = rosgraph_msgs::Log::DEBUG;
  test_subject->RecordLogs(MessageToConstptr(log_message_));

  rosgraph_msgs::Log log_message2 = log_message_;
  log_message2.level = rosgraph_msgs::Log::INFO;
  log_message2.name = node_name2;
  test_subject->RecordLogs(MessageToConstptr(log_message2));

  rosgraph_msgs::Log log_message3 = log_message_;
  log_message3.topics.push_back(topic1);
  log_message3.topics.push_back(topic2);
  log_message3.level = rosgraph_msgs::Log::WARN;
  test_subject->RecordLogs(MessageToConstptr(log_message3));
}

TEST_F(LogNodeFixture, TestRecordLogIgnoreList)
{
  std::unordered_set<std::string> ignore_nodes;
  ignore_nodes.emplace(log_message_.name);
  std::shared_ptr<LogNode> test_subject = BuildTestSubject(rosgraph_msgs::Log::DEBUG, ignore_nodes);

  InitializeLogNode(test_subject);

  ON_CALL(*log_service_, batchData(_))
    .WillByDefault(Return(true));
  EXPECT_CALL(*log_service_, batchData(_))
    .Times(0);

  log_message_.level = rosgraph_msgs::Log::INFO;
  test_subject->RecordLogs(MessageToConstptr(log_message_));
}

TEST_F(LogNodeFixture, Sanity) {
  ASSERT_TRUE(true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
