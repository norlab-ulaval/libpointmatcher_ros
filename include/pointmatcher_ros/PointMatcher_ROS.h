#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/IO.h>
#include <sensor_msgs/PointCloud2.h>

namespace PointMatcher_ROS
{
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg);

	template<typename T>
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);
}
