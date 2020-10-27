// google test
#include <gtest/gtest.h>

// std
#include <random>

// eigen
#include <Eigen/Dense>

// sensor_msgs
#include <sensor_msgs/PointCloud2.h>

// pointmacher_ros
#include "pointmatcher_ros/RosPointCloud2Deserializer.h"
#include "pointmatcher_ros/PointMatcher_ROS.h"

namespace PointMatcher_ROS
{

class RosPointCloud2DeserializerTest : public ::testing::Test
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ScalarType = float;
    using PM = PointMatcher<ScalarType>;
    using DataPoints = typename PM::DataPoints;
    using Label = typename DataPoints::Label;
    using Labels = typename DataPoints::Labels;
    using View = typename DataPoints::View;

    struct PointCloudMsgGenerationParameters
    {
        size_t nbPoints_{ 100 };
        size_t dimFeatures_{ 4 };
        size_t dimNormals_{ 0 };
        size_t dimColor_{ 0 };
        size_t dimScalar_{ 0 };
        size_t dimTime_{ 0 };

        bool hasNormals_{ false };
        bool hasColorRgb_{ false };
        bool hasColorRgba_{ false };
        bool hasScalar_{ false };
        bool hasScalarRangeRestricted_{ false };
        bool hasIntegerTime_{ false };

        std::string frameId_{ "sensor" };
        ros::Time stamp_;
    };

    RosPointCloud2DeserializerTest() = default;

    void SetUp() override
    {
        const int randomSeed = std::chrono::system_clock::now().time_since_epoch().count();
        randomNumberGenerator_.seed(randomSeed);
        RecordProperty("random_seed", randomSeed);

        unitaryUnsignedIntegerDistribution_.param(std::uniform_real_distribution<ScalarType>::param_type(0.0, 255.0));
    }

    /* Setup methods */
    DataPoints generatePointCloud(const PointCloudMsgGenerationParameters& parameters)
    {
        // Point features.
        Labels featLabels;
        const PM::Matrix features = PM::Matrix::Random(parameters.dimFeatures_, parameters.nbPoints_);
        featLabels.push_back(Label("x", 1));
        featLabels.push_back(Label("y", 1));
        featLabels.push_back(Label("z", 1));
        featLabels.push_back(Label("pad", 1));

        // Construct the point cloud from the generated matrices
        DataPoints pointCloud(features, featLabels);

        // Set padding/scale value to 1.
        pointCloud.getFeatureViewByName("pad").setOnes();

        // Normals.
        if (parameters.hasNormals_)
        {
            const PM::Matrix descriptor = PM::Matrix::Random(parameters.dimNormals_, parameters.nbPoints_);
            pointCloud.addDescriptor("normals", descriptor);
        }

        if (parameters.hasColorRgb_)
        {
            const PM::Matrix descriptor = PM::Matrix::Random(parameters.dimColor_, parameters.nbPoints_);
            pointCloud.addDescriptor("color", descriptor);
        }

        if (parameters.hasColorRgba_)
        {
            const PM::Matrix descriptor = PM::Matrix::Random(parameters.dimColor_, parameters.nbPoints_);
            pointCloud.addDescriptor("color", descriptor);
        }

        if (parameters.hasScalar_)
        {
            // Scalar descriptor in floating point range [0, 1].
            for (size_t i = 0; i < parameters.dimScalar_; i++)
            {
                const PM::Matrix descriptor = PM::Matrix::Random(1, parameters.nbPoints_);
                const std::string descriptorLabel("scalar" + std::to_string(i));
                pointCloud.addDescriptor(descriptorLabel, descriptor);
            }
        }
        else if (parameters.hasScalarRangeRestricted_)
        {
            for (size_t i = 0; i < parameters.dimScalar_; i++)
            {
                // Scalar descriptor in uint range [0, 255] represented as floating point. (e.g. LIDAR intensity and reflectivity)
                const PM::Matrix descriptor = PM::Matrix::Zero(1, parameters.nbPoints_).unaryExpr([&](float /*dummy*/) {
                    return unitaryUnsignedIntegerDistribution_(randomNumberGenerator_);
                });
                const std::string descriptorLabel("scalarRangeRestricted" + std::to_string(i));
                pointCloud.addDescriptor(descriptorLabel, descriptor);
            }
        }

        if (parameters.hasIntegerTime_)
        {
            // Vector descriptor in uint range [0, 255]. (e.g. seconds and nanoseconds)
            const PM::Int64Matrix times = PM::Int64Matrix::Random(parameters.dimTime_, parameters.nbPoints_);
            pointCloud.addTime("time", times);
        }

        return pointCloud;
    }

    // Random distribution.
    std::default_random_engine randomNumberGenerator_;
    std::uniform_real_distribution<ScalarType> unitaryUnsignedIntegerDistribution_;
};

TEST_F(RosPointCloud2DeserializerTest, EmptyRosMessage)
{
    // Ros sensor_msgs/PointCloud2 is empty by default.
    // Block motivation: Check base non-initialized values from msg. If at some point the message definition changes and this fails, it will be easier to know why.
    const sensor_msgs::PointCloud2 rosMsg;
    ASSERT_EQ(rosMsg.header.stamp.toSec(), 0);
    ASSERT_EQ(rosMsg.header.frame_id, "");
    ASSERT_EQ(rosMsg.fields.size(), 0u);
    ASSERT_FALSE(rosMsg.is_dense);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);

    // Assertions.
    ASSERT_EQ(deserializedPointCloud.getNbPoints(), 0u);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), 0u);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), 0u);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 0u);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), 0u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), 0u);
}

TEST_F(RosPointCloud2DeserializerTest, PointCloud3dPoints)
{
    PointCloudMsgGenerationParameters parameters;
    parameters.nbPoints_ = 1123455;

    // Compute dimension of descriptors.
    const size_t dimDescriptors = (parameters.dimNormals_ + parameters.dimColor_ + parameters.dimScalar_);

    // Create point cloud.
    const DataPoints pointCloud = generatePointCloud(parameters);
    const sensor_msgs::PointCloud2 rosMsg =
        PointMatcher_ROS::pointMatcherCloudToRosMsg<ScalarType>(pointCloud, parameters.frameId_, parameters.stamp_);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);

    // Assertions.
    EXPECT_EQ(deserializedPointCloud.getNbPoints(), parameters.nbPoints_);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), parameters.dimFeatures_ - 1);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), parameters.dimFeatures_);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 0u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), dimDescriptors);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), parameters.dimTime_);
    EXPECT_EQ(deserializedPointCloud, pointCloud);
}

TEST_F(RosPointCloud2DeserializerTest, PointCloud3dPointsNormals)
{
    PointCloudMsgGenerationParameters parameters;
    parameters.nbPoints_ = 1867565;
    parameters.dimNormals_ = 3;
    parameters.hasNormals_ = true;

    // Compute dimension of descriptors.
    const size_t dimDescriptors = (parameters.dimNormals_ + parameters.dimColor_ + parameters.dimScalar_);

    // Create point cloud.
    const DataPoints pointCloud = generatePointCloud(parameters);
    const sensor_msgs::PointCloud2 rosMsg =
        PointMatcher_ROS::pointMatcherCloudToRosMsg<ScalarType>(pointCloud, parameters.frameId_, parameters.stamp_);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);

    // Assertions.
    EXPECT_EQ(deserializedPointCloud.getNbPoints(), parameters.nbPoints_);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), parameters.dimFeatures_ - 1);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), parameters.dimFeatures_);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 1u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), dimDescriptors);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), parameters.dimTime_);
    EXPECT_EQ(deserializedPointCloud, pointCloud);
}

TEST_F(RosPointCloud2DeserializerTest, PointCloud3dPointsColor)
{
    PointCloudMsgGenerationParameters parameters;
    parameters.nbPoints_ = 15;
    parameters.dimColor_ = 3;
    parameters.hasColorRgb_ = true;

    // Compute dimension of descriptors.
    const size_t dimDescriptors = (parameters.dimNormals_ + parameters.dimColor_ + parameters.dimScalar_);

    // Create point cloud.
    const DataPoints pointCloud = generatePointCloud(parameters);
    const sensor_msgs::PointCloud2 rosMsg =
        PointMatcher_ROS::pointMatcherCloudToRosMsg<ScalarType>(pointCloud, parameters.frameId_, parameters.stamp_);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);
    // const DataPoints deserializedPointCloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<ScalarType>(rosMsg);

    // Assertions.
    EXPECT_EQ(deserializedPointCloud.getNbPoints(), parameters.nbPoints_);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), parameters.dimFeatures_ - 1);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), parameters.dimFeatures_);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 1u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), dimDescriptors);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), parameters.dimTime_);
    // TODO(ynava) Numerical accuracy of color type conversion makes it difficult to test for equality.
    // Improve deserialization and serialization to decrease inaccuracy.
    // EXPECT_TRUE(deserializedPointCloud.getDescriptorViewByName("color").isApprox(pointCloud.getDescriptorViewByName("color")));
}

TEST_F(RosPointCloud2DeserializerTest, PointCloud3dPointsColorWithTransparency)
{
    PointCloudMsgGenerationParameters parameters;
    parameters.nbPoints_ = 15;
    parameters.dimColor_ = 4;
    parameters.hasColorRgba_ = true;

    // Compute dimension of descriptors.
    const size_t dimDescriptors = (parameters.dimNormals_ + parameters.dimColor_ + parameters.dimScalar_);

    // Create point cloud.
    const DataPoints pointCloud = generatePointCloud(parameters);
    const sensor_msgs::PointCloud2 rosMsg =
        pointmatcher_ros::pointMatcherCloudToRosMsg<ScalarType>(pointCloud, parameters.frameId_, parameters.stamp_);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);

    // Assertions.
    EXPECT_EQ(deserializedPointCloud.getNbPoints(), parameters.nbPoints_);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), parameters.dimFeatures_ - 1);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), parameters.dimFeatures_);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 1u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), dimDescriptors);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), parameters.dimTime_);
    // TODO(ynava) Numerical accuracy of color type conversion makes it difficult to test for equality.
    // Improve deserialization and serialization to decrease inaccuracy.
    // EXPECT_TRUE(deserializedPointCloud.getDescriptorViewByName("color").isApprox(pointCloud.getDescriptorViewByName("color")));
}

TEST_F(RosPointCloud2DeserializerTest, PointCloud3dPointsScalar)
{
    PointCloudMsgGenerationParameters parameters;
    parameters.nbPoints_ = 99765;
    parameters.dimScalar_ = 1;
    parameters.hasScalar_ = true;

    // Compute dimension of descriptors.
    const size_t dimDescriptors = (parameters.dimNormals_ + parameters.dimColor_ + parameters.dimScalar_);

    // Create point cloud.
    const DataPoints pointCloud = generatePointCloud(parameters);
    const sensor_msgs::PointCloud2 rosMsg =
        PointMatcher_ROS::pointMatcherCloudToRosMsg<ScalarType>(pointCloud, parameters.frameId_, parameters.stamp_);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);

    // Assertions.
    EXPECT_EQ(deserializedPointCloud.getNbPoints(), parameters.nbPoints_);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), parameters.dimFeatures_ - 1);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), parameters.dimFeatures_);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 1u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), dimDescriptors);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), parameters.dimTime_);
    EXPECT_EQ(deserializedPointCloud, pointCloud);
}

TEST_F(RosPointCloud2DeserializerTest, PointCloud3dPointsScalarRangeRestricted)
{
    PointCloudMsgGenerationParameters parameters;
    parameters.nbPoints_ = 5456;
    parameters.dimScalar_ = 1;
    parameters.hasScalarRangeRestricted_ = true;

    // Compute dimension of descriptors.
    const size_t dimDescriptors = (parameters.dimNormals_ + parameters.dimColor_ + parameters.dimScalar_);

    // Create point cloud.
    const DataPoints pointCloud = generatePointCloud(parameters);
    const sensor_msgs::PointCloud2 rosMsg =
        PointMatcher_ROS::pointMatcherCloudToRosMsg<ScalarType>(pointCloud, parameters.frameId_, parameters.stamp_);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);

    // Assertions.
    EXPECT_EQ(deserializedPointCloud.getNbPoints(), parameters.nbPoints_);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), parameters.dimFeatures_ - 1);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), parameters.dimFeatures_);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 1u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), dimDescriptors);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), parameters.dimTime_);
    EXPECT_EQ(deserializedPointCloud, pointCloud);
}

TEST_F(RosPointCloud2DeserializerTest, PointCloud3dPointsNormalsScalars)
{
    PointCloudMsgGenerationParameters parameters;
    parameters.nbPoints_ = 465;
    parameters.dimNormals_ = 3;
    parameters.dimScalar_ = 5;
    parameters.hasNormals_ = true;
    parameters.hasScalar_ = true;

    // Compute dimension of descriptors.
    const size_t dimDescriptors = (parameters.dimNormals_ + parameters.dimColor_ + parameters.dimScalar_);

    // Create point cloud.
    const DataPoints pointCloud = generatePointCloud(parameters);
    const sensor_msgs::PointCloud2 rosMsg =
        PointMatcher_ROS::pointMatcherCloudToRosMsg<ScalarType>(pointCloud, parameters.frameId_, parameters.stamp_);

    // Deserialize msg.
    const DataPoints deserializedPointCloud = RosPointCloud2Deserializer<ScalarType>::deserialize(rosMsg);

    // Assertions.
    EXPECT_EQ(deserializedPointCloud.getNbPoints(), parameters.nbPoints_);
    EXPECT_EQ(deserializedPointCloud.getEuclideanDim(), parameters.dimFeatures_ - 1);
    EXPECT_EQ(deserializedPointCloud.getHomogeneousDim(), parameters.dimFeatures_);
    EXPECT_EQ(deserializedPointCloud.getNbGroupedDescriptors(), 6u);
    EXPECT_EQ(deserializedPointCloud.getDescriptorDim(), dimDescriptors);
    EXPECT_EQ(deserializedPointCloud.getTimeDim(), parameters.dimTime_);
    EXPECT_EQ(deserializedPointCloud, pointCloud);
}

} // namespace PointMatcher_ROS
