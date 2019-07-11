#include "pointmatcher_ros/PointMatcher_ROS.h"
#include "utils.h"
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>

template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher_ROS::rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg)
{
	typedef PointMatcher<T> PM;
	typedef typename PointMatcherIO<T>::PMPropTypes PM_types;
	typedef typename PM::DataPoints DataPoints;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	typedef typename DataPoints::View View;
	typedef typename DataPoints::TimeView TimeView;
	
	Labels featLabels;
	Labels descLabels;
	Labels timeLabels;
	std::vector<bool> isFeature;
	std::vector<PM_types> fieldTypes;
	for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it)
	{
		const std::string name(it->name);
		const size_t count(std::max<size_t>(it->count, 1));
		if(name == "x" || name == "y" || name == "z")
		{
			featLabels.push_back(Label(name, count));
			isFeature.push_back(true);
			fieldTypes.push_back(PM_types::FEATURE);
		}
		else if(name == "rgb" || name == "rgba")
		{
			descLabels.push_back(Label("color", (name == "rgba") ? 4 : 3));
			isFeature.push_back(false);
			fieldTypes.push_back(PM_types::DESCRIPTOR);
		}
		else if((it + 1) != rosMsg.fields.end() && it->name == "normal_x" && (it + 1)->name == "normal_y")
		{
			if((it + 2) != rosMsg.fields.end() && (it + 2)->name == "normal_z")
			{
				descLabels.push_back(Label("normals", 3));
				it += 2;
				isFeature.push_back(false);
				isFeature.push_back(false);
				fieldTypes.push_back(PM_types::DESCRIPTOR);
				fieldTypes.push_back(PM_types::DESCRIPTOR);
			}
			else
			{
				descLabels.push_back(Label("normals", 2));
				it += 1;
				isFeature.push_back(false);
				fieldTypes.push_back(PM_types::DESCRIPTOR);
			}
			isFeature.push_back(false);
			fieldTypes.push_back(PM_types::DESCRIPTOR);
		}
		else if((it + 1) != rosMsg.fields.end() && ends_with(name, "_splitTime_high32") && ends_with(((it + 1)->name), "_splitTime_low32"))
		{
			// time extraction
			std::string startingName = name;
			erase_last(startingName, "_splitTime_high32");
			const std::string beginning = startingName;
			
			timeLabels.push_back(Label(beginning, 1));
			it += 1;
			isFeature.push_back(false);
			fieldTypes.push_back(PM_types::TIME);
			fieldTypes.push_back(PM_types::TIME);
		}
		else if(name == "time")
		{
			timeLabels.push_back(Label(name, count));
			isFeature.push_back(false);
		}
		else
		{
			descLabels.push_back(Label(name, count));
			isFeature.push_back(false);
			fieldTypes.push_back(PM_types::DESCRIPTOR);
		}
	}
	
	featLabels.push_back(Label("pad", 1));
	assert(isFeature.size() == rosMsg.fields.size());
	assert(fieldTypes.size() == rosMsg.fields.size());
	
	// create cloud
	const unsigned pointCount(rosMsg.width * rosMsg.height);
	DataPoints cloud(featLabels, descLabels, timeLabels, pointCount);
	cloud.getFeatureViewByName("pad").setConstant(1);
	
	// fill cloud
	typedef sensor_msgs::PointField PF;
	size_t fieldId = 0;
	for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it, ++fieldId)
	{
		if(it->name == "rgb" || it->name == "rgba")
		{
			// special case for colors
			if(((it->datatype != PF::UINT32) && (it->datatype != PF::INT32) && (it->datatype != PF::FLOAT32)) || (it->count != 1))
			{
				throw std::runtime_error(
						"Colors in a point cloud must be a single element of size 32 bits, found " +
						std::to_string(it->count) + " elements of type " + std::to_string(unsigned(it->datatype))
				);
			}
			View view(cloud.getDescriptorViewByName("color"));
			int ptId(0);
			for(size_t y(0); y < rosMsg.height; ++y)
			{
				const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
				for(size_t x(0); x < rosMsg.width; ++x)
				{
					const uint32_t rgba(*reinterpret_cast<const uint32_t*>(dataPtr + it->offset));
					const T colorA(T((rgba >> 24) & 0xff) / 255.);
					const T colorR(T((rgba >> 16) & 0xff) / 255.);
					const T colorG(T((rgba >> 8) & 0xff) / 255.);
					const T colorB(T((rgba >> 0) & 0xff) / 255.);
					view(0, ptId) = colorR;
					view(1, ptId) = colorG;
					view(2, ptId) = colorB;
					if(view.rows() > 3)
					{
						view(3, ptId) = colorA;
					}
					dataPtr += rosMsg.point_step;
					ptId += 1;
				}
			}
		}
		else if(ends_with(it->name, "_splitTime_high32") || ends_with(it->name, "_splitTime_low32"))
		{
			std::string startingName = it->name;
			bool isHigh = false;
			if(ends_with(it->name, "_splitTime_high32"))
			{
				erase_last(startingName, "_splitTime_high32");
				isHigh = true;
			}
			if(ends_with(it->name, "_splitTime_low32"))
			{
				erase_last(startingName, "_splitTime_low32");
			}
			
			TimeView timeView(cloud.getTimeViewByName(startingName));
			
			int ptId(0);
			const size_t count(std::max<size_t>(it->count, 1));
			for(size_t y(0); y < rosMsg.height; ++y)
			{
				const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
				for(size_t x(0); x < rosMsg.width; ++x)
				{
					const uint8_t* fPtr(dataPtr + it->offset);
					for(unsigned dim(0); dim < count; ++dim)
					{
						if(isHigh)
						{
							const uint32_t high32 = *reinterpret_cast<const uint32_t*>(fPtr);
							const uint32_t low32 = uint32_t(timeView(dim, ptId));
							timeView(dim, ptId) = (((uint64_t)high32) << 32) | ((uint64_t)low32);
						}
						else
						{
							const uint32_t high32 = uint32_t(timeView(dim, ptId) >> 32);
							const uint32_t low32 = *reinterpret_cast<const uint32_t*>(fPtr);
							timeView(dim, ptId) = (((uint64_t)high32) << 32) | ((uint64_t)low32);
							
						}
						dataPtr += rosMsg.point_step;
						ptId += 1;
					}
				}
			}
			
		}
		else
		{
			
			// get view for editing data
			View view(
					(it->name == "normal_x") ? cloud.getDescriptorRowViewByName("normals", 0) :
					((it->name == "normal_y") ? cloud.getDescriptorRowViewByName("normals", 1) :
					 ((it->name == "normal_z") ? cloud.getDescriptorRowViewByName("normals", 2) :
					  ((isFeature[fieldId]) ? cloud.getFeatureViewByName(it->name) :
					   cloud.getDescriptorViewByName(it->name))))
			);
			
			// use view to read data
			int ptId(0);
			const size_t count(std::max<size_t>(it->count, 1));
			for(size_t y(0); y < rosMsg.height; ++y)
			{
				const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
				for(size_t x(0); x < rosMsg.width; ++x)
				{
					const uint8_t* fPtr(dataPtr + it->offset);
					for(unsigned dim(0); dim < count; ++dim)
					{
						switch(it->datatype)
						{
							case PF::INT8:
								view(dim, ptId) = T(*reinterpret_cast<const int8_t*>(fPtr));
								fPtr += 1;
								break;
							case PF::UINT8:
								view(dim, ptId) = T(*reinterpret_cast<const uint8_t*>(fPtr));
								fPtr += 1;
								break;
							case PF::INT16:
								view(dim, ptId) = T(*reinterpret_cast<const int16_t*>(fPtr));
								fPtr += 2;
								break;
							case PF::UINT16:
								view(dim, ptId) = T(*reinterpret_cast<const uint16_t*>(fPtr));
								fPtr += 2;
								break;
							case PF::INT32:
								view(dim, ptId) = T(*reinterpret_cast<const int32_t*>(fPtr));
								fPtr += 4;
								break;
							case PF::UINT32:
								view(dim, ptId) = T(*reinterpret_cast<const uint32_t*>(fPtr));
								fPtr += 4;
								break;
							case PF::FLOAT32:
								view(dim, ptId) = T(*reinterpret_cast<const float*>(fPtr));
								fPtr += 4;
								break;
							case PF::FLOAT64:
								view(dim, ptId) = T(*reinterpret_cast<const double*>(fPtr));
								fPtr += 8;
								break;
							default:
								abort();
						}
					}
					dataPtr += rosMsg.point_step;
					ptId += 1;
				}
			}
		}
	}
	
	return cloud;
}

template
PointMatcher<float>::DataPoints PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(const sensor_msgs::PointCloud2& rosMsg);

template
PointMatcher<double>::DataPoints PointMatcher_ROS::rosMsgToPointMatcherCloud<double>(const sensor_msgs::PointCloud2& rosMsg);

template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher_ROS::rosMsgToPointMatcherCloud(const sensor_msgs::LaserScan& rosMsg)
{
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	typedef typename DataPoints::View View;
	
	Labels featLabels;
	featLabels.push_back(Label("x", 1));
	featLabels.push_back(Label("y", 1));
	featLabels.push_back(Label("pad", 1));
	
	// Build descriptors
	Labels descLabels;
	if(!rosMsg.intensities.empty())
	{
		descLabels.push_back(Label("intensity", 1));
		assert(rosMsg.intensities.size() == rosMsg.ranges.size());
	}
	
	// filter points based on range
	std::vector<size_t> ids(rosMsg.ranges.size());
	std::vector<double> ranges(rosMsg.ranges.size());
	std::vector<double> intensities(rosMsg.intensities.size());
	
	size_t goodCount(0);
	for(size_t i = 0; i < rosMsg.ranges.size(); ++i)
	{
		const float range(rosMsg.ranges[i]);
		if(range >= rosMsg.range_min && range <= rosMsg.range_max)
		{
			ranges[goodCount] = range;
			ids[goodCount] = i;
			if(!rosMsg.intensities.empty())
			{
				intensities[goodCount] = rosMsg.intensities[i];
			}
			++goodCount;
		}
	}
	
	ids.resize(goodCount);
	ranges.resize(goodCount);
	if(!rosMsg.intensities.empty())
	{
		intensities.resize(goodCount);
	}
	
	DataPoints cloud(featLabels, descLabels, goodCount);
	cloud.getFeatureViewByName("pad").setConstant(1);
	
	// fill features
	for(size_t i = 0; i < ranges.size(); ++i)
	{
		const T angle = rosMsg.angle_min + ids[i] * rosMsg.angle_increment;
		const T range(ranges[i]);
		
		cloud.features(0, i) = cos(angle) * range;
		cloud.features(1, i) = sin(angle) * range;
	}
	
	// fill descriptors
	if(!rosMsg.intensities.empty())
	{
		auto is(cloud.getDescriptorViewByName("intensity"));
		for(size_t i = 0; i < intensities.size(); ++i)
		{
			is(0, i) = intensities[i];
		}
	}
	
	return cloud;
}

template
PointMatcher<float>::DataPoints PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(const sensor_msgs::LaserScan& rosMsg);

template
PointMatcher<double>::DataPoints PointMatcher_ROS::rosMsgToPointMatcherCloud<double>(const sensor_msgs::LaserScan& rosMsg);

template<typename T>
sensor_msgs::PointCloud2 PointMatcher_ROS::pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id,
																	 const ros::Time& stamp)
{
	
	sensor_msgs::PointCloud2 rosCloud;
	typedef sensor_msgs::PointField PF;
	
	// check type and get sizes
	assert(std::is_floating_point<T>::value);
	assert(!(std::is_same<T, long double>::value));
	uint8_t dataType;
	size_t scalarSize;
	if(typeid(T) == typeid(float))
	{
		dataType = PF::FLOAT32;
		scalarSize = 4;
	}
	else
	{
		dataType = PF::FLOAT64;
		scalarSize = 8;
	}
	
	size_t timeSize = 4; // we split in two UINT32
	
	// build labels
	
	// features
	unsigned offset(0);
	assert(!pmCloud.featureLabels.empty());
	assert(pmCloud.featureLabels[pmCloud.featureLabels.size() - 1].text == "pad");
	for(auto it(pmCloud.featureLabels.begin()); it != pmCloud.featureLabels.end(); ++it)
	{
		// last label is padding
		if((it + 1) == pmCloud.featureLabels.end())
		{
			break;
		}
		PF pointField;
		pointField.name = it->text;
		pointField.offset = offset;
		pointField.datatype = dataType;
		pointField.count = it->span;
		rosCloud.fields.push_back(pointField);
		offset += it->span * scalarSize;
	}
	bool addZ(false);
	if(!pmCloud.featureLabels.contains("z"))
	{
		PF pointField;
		pointField.name = "z";
		pointField.offset = offset;
		pointField.datatype = dataType;
		pointField.count = 1;
		rosCloud.fields.push_back(pointField);
		offset += scalarSize;
		addZ = true;
	}
	
	// descriptors
	const bool isDescriptor(!pmCloud.descriptorLabels.empty());
	bool hasColor(false);
	unsigned colorPos(0);
	unsigned colorCount(0);
	unsigned inDescriptorPos(0);
	for(auto it(pmCloud.descriptorLabels.begin()); it != pmCloud.descriptorLabels.end(); ++it)
	{
		PF pointField;
		if(it->text == "normals")
		{
			assert((it->span == 2) || (it->span == 3));
			pointField.datatype = dataType;
			pointField.name = "normal_x";
			pointField.offset = offset;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += scalarSize;
			pointField.name = "normal_y";
			pointField.offset = offset;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += scalarSize;
			if(it->span == 3)
			{
				pointField.name = "normal_z";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += scalarSize;
			}
		}
		else if(it->text == "color")
		{
			colorPos = inDescriptorPos;
			colorCount = it->span;
			hasColor = true;
			pointField.datatype = (colorCount == 4) ? uint8_t(PF::UINT32) : uint8_t(PF::FLOAT32);
			pointField.name = (colorCount == 4) ? "rgba" : "rgb";
			pointField.offset = offset;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += 4;
		}
		else
		{
			pointField.datatype = dataType;
			pointField.name = it->text;
			pointField.offset = offset;
			pointField.count = it->span;
			rosCloud.fields.push_back(pointField);
			offset += it->span * scalarSize;
		}
		inDescriptorPos += it->span;
	}
	
	// time
	bool hasTime(false);
	for(auto it(pmCloud.timeLabels.begin()); it != pmCloud.timeLabels.end(); ++it)
	{
		PF pointField;
		if(it->text == "time")
		{
			hasTime = true;
			
			// for Rviz view
			
			pointField.datatype = PF::FLOAT32;
			
			pointField.name = "elapsedTimeSec";
			pointField.offset = offset;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += 4;
			
			// Split time in two because there is not PF::UINT64
			pointField.datatype = PF::UINT32;
			pointField.name = it->text + "_splitTime_high32";
			pointField.offset = offset;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += timeSize;
			
			pointField.datatype = PF::UINT32;
			pointField.name = it->text + "_splitTime_low32";
			pointField.offset = offset;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += timeSize;
		}
	}
	
	// fill cloud with data
	rosCloud.header.frame_id = frame_id;
	rosCloud.header.stamp = stamp;
	rosCloud.height = 1;
	rosCloud.width = pmCloud.features.cols();
	rosCloud.is_bigendian = false;
	rosCloud.point_step = offset;
	rosCloud.row_step = rosCloud.point_step * rosCloud.width;
	rosCloud.is_dense = true;
	rosCloud.data.resize(rosCloud.row_step * rosCloud.height);
	
	const unsigned featureDim(pmCloud.features.rows() - 1);
	const unsigned descriptorDim(pmCloud.descriptors.rows());
	const unsigned timeDim(pmCloud.times.rows());
	
	assert(descriptorDim == inDescriptorPos);
	const unsigned postColorPos(colorPos + colorCount);
	assert(postColorPos <= inDescriptorPos);
	const unsigned postColorCount(descriptorDim - postColorPos);
	
	for(unsigned pt(0); pt < rosCloud.width; ++pt)
	{
		uint8_t* fPtr(&rosCloud.data[pt * offset]);
		
		memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.features(0, pt)), scalarSize * featureDim);
		fPtr += scalarSize * featureDim;
		if(addZ)
		{
			memset(fPtr, 0, scalarSize);
			fPtr += scalarSize;
		}
		if(isDescriptor)
		{
			if(hasColor)
			{
				// before color
				memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * colorPos);
				fPtr += scalarSize * colorPos;
				// compact color
				uint32_t rgba;
				unsigned colorR(unsigned(pmCloud.descriptors(colorPos + 0, pt) * 255.) & 0xFF);
				unsigned colorG(unsigned(pmCloud.descriptors(colorPos + 1, pt) * 255.) & 0xFF);
				unsigned colorB(unsigned(pmCloud.descriptors(colorPos + 2, pt) * 255.) & 0xFF);
				unsigned colorA(0);
				if(colorCount == 4)
				{
					colorA = unsigned(pmCloud.descriptors(colorPos + 3, pt) * 255.) & 0xFF;
				}
				rgba = colorA << 24 | colorR << 16 | colorG << 8 | colorB;
				memcpy(fPtr, reinterpret_cast<const uint8_t*>(&rgba), 4);
				fPtr += 4;
				// after color
				memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(postColorPos, pt)), scalarSize * postColorCount);
				fPtr += scalarSize * postColorCount;
			}
			else
			{
				memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * descriptorDim);
				fPtr += scalarSize * descriptorDim;
			}
		}
		
		if(hasTime)
		{
			const size_t ptrSize = timeSize * timeDim;
			
			// Elapsed time
			const float elapsedTime = (float)(pmCloud.times(0, pt) - pmCloud.times(0, 0)) * 1e-9f;
			memcpy(fPtr, reinterpret_cast<const uint8_t*>(&elapsedTime), ptrSize);
			fPtr += ptrSize;
			
			// high32
			const uint32_t high32 = (uint32_t)(pmCloud.times(0, pt) >> 32);
			memcpy(fPtr, reinterpret_cast<const uint8_t*>(&high32), ptrSize);
			fPtr += ptrSize;
			
			// low32
			const uint32_t low32 = (uint32_t)(pmCloud.times(0, pt));
			memcpy(fPtr, reinterpret_cast<const uint8_t*>(&low32), ptrSize);
			fPtr += ptrSize;
			
		}
	}
	
	// fill remaining information
	rosCloud.header.frame_id = frame_id;
	rosCloud.header.stamp = stamp;
	
	return rosCloud;
}

template
sensor_msgs::PointCloud2 PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(const PointMatcher<float>::DataPoints& pmCloud, const std::string& frame_id,
																			const ros::Time& stamp);

template
sensor_msgs::PointCloud2 PointMatcher_ROS::pointMatcherCloudToRosMsg<double>(const PointMatcher<double>::DataPoints& pmCloud, const std::string& frame_id,
																			 const ros::Time& stamp);

template<typename T>
nav_msgs::Odometry PointMatcher_ROS::pointMatcherTransformationToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr,
																		 const std::string& frame_id, const ros::Time& stamp)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = stamp;
	odom.header.frame_id = frame_id;
	
	// Fill pose
	const Eigen::Affine3d eigenTr(
			Eigen::Matrix4d(
					matrixToDim<double>(
							inTr.template cast<double>(), 4
					)
			)
	);
	odom.pose.pose = tf2::toMsg(eigenTr);
	
	// Fill velocity
	odom.twist.covariance[0 + 0 * 6] = -1;
	odom.twist.covariance[1 + 1 * 6] = -1;
	odom.twist.covariance[2 + 2 * 6] = -1;
	odom.twist.covariance[3 + 3 * 6] = -1;
	odom.twist.covariance[4 + 4 * 6] = -1;
	odom.twist.covariance[5 + 5 * 6] = -1;
	
	return odom;
}

template
nav_msgs::Odometry PointMatcher_ROS::pointMatcherTransformationToOdomMsg<float>(const PointMatcher<float>::TransformationParameters& inTr,
																				const std::string& frame_id, const ros::Time& stamp);

template
nav_msgs::Odometry PointMatcher_ROS::pointMatcherTransformationToOdomMsg<double>(const PointMatcher<double>::TransformationParameters& inTr,
																				 const std::string& frame_id, const ros::Time& stamp);

template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher_ROS::rosTfToPointMatcherTransformation(const geometry_msgs::TransformStamped& transformStamped,
																									   const int& transformationDimension)
{
	Eigen::Affine3d eigenTr = tf2::transformToEigen(transformStamped);
	return matrixToDim<T>(eigenTr.matrix().cast<T>(), transformationDimension);
}

template
PointMatcher<float>::TransformationParameters
PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(const geometry_msgs::TransformStamped& transformStamped, const int& transformationDimension);

template
PointMatcher<double>::TransformationParameters
PointMatcher_ROS::rosTfToPointMatcherTransformation<double>(const geometry_msgs::TransformStamped& transformStamped, const int& transformationDimension);

template<typename T>
geometry_msgs::TransformStamped PointMatcher_ROS::pointMatcherTransformationToRosTf(const typename PointMatcher<T>::TransformationParameters& inTr,
																					const std::string& frame_id, const std::string& child_frame_id,
																					const ros::Time& stamp)
{
	const Eigen::Affine3d eigenTr(
			Eigen::Matrix4d(
					matrixToDim<double>(
							inTr.template cast<double>(), 4
					)
			)
	);
	geometry_msgs::TransformStamped tfTr = tf2::eigenToTransform(eigenTr);
	tfTr.header.frame_id = frame_id;
	tfTr.child_frame_id = child_frame_id;
	tfTr.header.stamp = stamp;
	return tfTr;
}

template
geometry_msgs::TransformStamped PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(const PointMatcher<float>::TransformationParameters& inTr,
																						   const std::string& frame_id,
																						   const std::string& child_frame_id, const ros::Time& stamp);

template
geometry_msgs::TransformStamped PointMatcher_ROS::pointMatcherTransformationToRosTf<double>(const PointMatcher<double>::TransformationParameters& inTr,
																							const std::string& frame_id,
																							const std::string& child_frame_id, const ros::Time& stamp);
