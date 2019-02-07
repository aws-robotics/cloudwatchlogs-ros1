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
#include <cloudwatch_logger/log_node_param_helper.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <aws_common/sdk_utils/aws_error.h>

using namespace Aws::Client;
using namespace Aws::CloudWatchLogs::Utils;
using ::testing::_;
using ::testing::Eq;
using ::testing::A;
using ::testing::InSequence;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::Return;
using Aws::AwsError;

class ParameterReaderMock : public ParameterReaderInterface 
{
public:
 MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::vector<std::string> &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, double &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, int &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, bool &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, Aws::String &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::string &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::map<std::string, std::string> &));
};

class LogNodeParamHelperFixture : public ::testing::Test
{
protected:
  std::shared_ptr<ParameterReaderMock> param_reader_ = std::make_shared<ParameterReaderMock>();
};

TEST_F(LogNodeParamHelperFixture, TestReadPublishFrequency)
{
    double expected_param_value = 42.0;

    {
      InSequence read_param_seq;

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamPublishFrequencyKey)), A<double&>()))
        .WillOnce(Return(AwsError::AWS_ERR_FAILURE)); 

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamPublishFrequencyKey)), A<double&>()))
        .WillOnce(Return(AwsError::AWS_ERR_NOT_FOUND)); 

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamPublishFrequencyKey)), A<double&>()))
        .WillOnce(
          DoAll(SetArgReferee<1>(expected_param_value), Return(AwsError::AWS_ERR_OK))
        );
    }

    double param = -1; 
    EXPECT_EQ(AwsError::AWS_ERR_FAILURE, ReadPublishFrequency(param_reader_, param));
    EXPECT_EQ(kNodePublishFrequencyDefaultValue, param);

    param = -1; 
    EXPECT_EQ(AwsError::AWS_ERR_NOT_FOUND, ReadPublishFrequency(param_reader_, param));
    EXPECT_EQ(kNodePublishFrequencyDefaultValue, param);
    
    param = -1; 
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadPublishFrequency(param_reader_, param));
    EXPECT_EQ(expected_param_value, param);
}

TEST_F(LogNodeParamHelperFixture, TestReadLogGroup)
{
    std::string expected_param_value = "MTc4MGNjOTc3ZTA1OTY";

    {
      InSequence read_param_seq;

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamLogGroupNameKey)), A<std::string &>()))
        .WillOnce(Return(AwsError::AWS_ERR_FAILURE));

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamLogGroupNameKey)), A<std::string &>()))
        .WillOnce(Return(AwsError::AWS_ERR_NOT_FOUND));

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamLogGroupNameKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>(expected_param_value), Return(AwsError::AWS_ERR_OK))
        );
    }

    std::string param = ""; 
    EXPECT_EQ(AwsError::AWS_ERR_FAILURE, ReadLogGroup(param_reader_, param));
    EXPECT_STREQ(kNodeLogGroupNameDefaultValue, param.c_str());

    param = ""; 
    EXPECT_EQ(AwsError::AWS_ERR_NOT_FOUND, ReadLogGroup(param_reader_, param));
    EXPECT_STREQ(kNodeLogGroupNameDefaultValue, param.c_str());
    
    param = ""; 
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadLogGroup(param_reader_, param));
    EXPECT_STREQ(expected_param_value.c_str(), param.c_str());
}

TEST_F(LogNodeParamHelperFixture, TestReadLogStream)
{
    std::string expected_param_value = "MGQyNTU3N2I3NzU1ZGIyNWQzMTZhYmVh";

    {
      InSequence read_param_seq;

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamLogStreamNameKey)), A<std::string &>()))
        .WillOnce(Return(AwsError::AWS_ERR_FAILURE));

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamLogStreamNameKey)), A<std::string &>()))
        .WillOnce(Return(AwsError::AWS_ERR_NOT_FOUND));

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamLogStreamNameKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>(expected_param_value), Return(AwsError::AWS_ERR_OK))
        );
    }

    std::string param = ""; 
    EXPECT_EQ(AwsError::AWS_ERR_FAILURE, ReadLogStream(param_reader_, param));
    EXPECT_STREQ(kNodeLogStreamNameDefaultValue, param.c_str());

    param = ""; 
    EXPECT_EQ(AwsError::AWS_ERR_NOT_FOUND, ReadLogStream(param_reader_, param));
    EXPECT_STREQ(kNodeLogStreamNameDefaultValue, param.c_str());
    
    param = ""; 
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadLogStream(param_reader_, param));
    EXPECT_STREQ(expected_param_value.c_str(), param.c_str());
}

TEST_F(LogNodeParamHelperFixture, TestReadSubscribeToRosout)
{
    bool expected_param_value = ! kNodeSubscribeToRosoutDefaultValue;

    {
      InSequence read_param_seq;

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamSubscribeToRosoutKey)), A<bool &>()))
        .WillOnce(Return(AwsError::AWS_ERR_FAILURE));

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamSubscribeToRosoutKey)), A<bool &>()))
        .WillOnce(Return(AwsError::AWS_ERR_NOT_FOUND));

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamSubscribeToRosoutKey)), A<bool &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>(expected_param_value), Return(AwsError::AWS_ERR_OK))
        );
    }

    bool param = ! kNodeSubscribeToRosoutDefaultValue;
    EXPECT_EQ(AwsError::AWS_ERR_FAILURE, ReadSubscribeToRosout(param_reader_, param));
    EXPECT_EQ(kNodeSubscribeToRosoutDefaultValue, param);

    param = ! kNodeSubscribeToRosoutDefaultValue;; 
    EXPECT_EQ(AwsError::AWS_ERR_NOT_FOUND, ReadSubscribeToRosout(param_reader_, param));
    EXPECT_EQ(kNodeSubscribeToRosoutDefaultValue, param);
    
    param = ! kNodeSubscribeToRosoutDefaultValue; 
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadSubscribeToRosout(param_reader_, param));
    EXPECT_EQ(expected_param_value, param);
}

TEST_F(LogNodeParamHelperFixture, TestReadReadMinLogVerbosity)
{
    {
      InSequence read_param_seq;

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(Return(AwsError::AWS_ERR_FAILURE));

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(Return(AwsError::AWS_ERR_NOT_FOUND)); 

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>("DEBUG"), Return(AwsError::AWS_ERR_OK))
        );

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>("INFO"), Return(AwsError::AWS_ERR_OK))
        );

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>("WARN"), Return(AwsError::AWS_ERR_OK))
        );

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>("ERROR"), Return(AwsError::AWS_ERR_OK))
        );

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>("FATAL"), Return(AwsError::AWS_ERR_OK))
        );

      EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMinLogVerbosityKey)), A<std::string &>()))
        .WillOnce(
          DoAll(SetArgReferee<1>("xNDlhZmJmNGM"), Return(AwsError::AWS_ERR_OK))
        );      
    }

    int8_t param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_FAILURE, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(kNodeMinLogVerbosityDefaultValue, param);

    param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_NOT_FOUND, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(kNodeMinLogVerbosityDefaultValue, param);

    param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(rosgraph_msgs::Log::DEBUG, param);

    param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(rosgraph_msgs::Log::INFO, param);

    param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(rosgraph_msgs::Log::WARN, param);

    param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(rosgraph_msgs::Log::ERROR, param);

    param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_OK, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(rosgraph_msgs::Log::FATAL, param);

    param = rosgraph_msgs::Log::FATAL + 1 ;
    EXPECT_EQ(AwsError::AWS_ERR_PARAM, ReadMinLogVerbosity(param_reader_, param));
    EXPECT_EQ(kNodeMinLogVerbosityDefaultValue, param);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
