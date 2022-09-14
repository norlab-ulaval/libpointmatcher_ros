#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/IO.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace PointMatcher_ROS
{
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::msg::PointCloud2& rosMsg);
	
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::msg::LaserScan& rosMsg);

	template<typename T>
	sensor_msgs::msg::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id,
													   const rclcpp::Time& stamp);

	template<typename T>
	nav_msgs::msg::Odometry pointMatcherTransformationToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& frame_id,
														   const std::string& child_frame_id, const rclcpp::Time& stamp);

	template<typename T>
	typename PointMatcher<T>::TransformationParameters rosTfToPointMatcherTransformation(const geometry_msgs::msg::TransformStamped& transformStamped,
																						 const int& transformationDimension);

	template<typename T>
	geometry_msgs::msg::TransformStamped pointMatcherTransformationToRosTf(const typename PointMatcher<T>::TransformationParameters& inTr,
																	  const std::string& frame_id, const std::string& child_frame_id, const rclcpp::Time& stamp);

    template<typename T>
    typename PointMatcher<T>::TransformationParameters rosMsgToPointMatcherTransformation(const geometry_msgs::msg::Pose& pose, const int& transformationDimension);
}
