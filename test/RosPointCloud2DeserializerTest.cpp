/*
 * RosPointCloud2DeserializerTest.cpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */

// google test
#include <gtest/gtest.h>

// eigen
#include <Eigen/Dense>

// sensor_msgs
#include <sensor_msgs/PointCloud2.h>

// pointmacher_ros
#include "pointmatcher_ros/RosPointCloud2Deserializer.h"

namespace PointMatcher_ROS
{

class RosPointCloud2DeserializerTest : public ::testing::Test
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RosPointCloud2DeserializerTest() = default;

    /* Setup methods */
    void createDefaultPointCloudMessages() {}

    // Error tolerance.
    const float kEpsilonError_ = 1e-5;
};

// This test validates that the function that builds up transformations to point clouds is correct. Considers pure translation
TEST_F(RosPointCloud2DeserializerTest, EmptyRosMessage)
{}


} // namespace PointMatcher_ROS
