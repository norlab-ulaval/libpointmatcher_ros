# libpointmatcher_ros
A bridge between libpointmatcher and ROS, converts between [ROS point cloud messages](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html) and libpointmatcher `DataPoints` objects.
While the conversion of most datafields is straightforward, the per-point time field if more complicated due to different manufactures/driver authors represent timestamp messages.
In this driver, we distinguish between three types of timestamps:

| Name         | Tested on                     | Datatype | ROS PointField | Stamped with |
|--------------|-------------------------------|----------|----------------|--------------|
| Ouster-like  | RS-32                         | uint32   | 6              | beginning    |
| Leishen-like | LS L128 S1                    | float32  | 7              | end          |
| Hesai-like   | RS Ruby Plus, Hesai Pandar XT | float64  | 8              | beginning    |

Inside libpointmatcher, all timestamps are represented in a field called `time` as `int64` datatypes.
This usually represent a timestamp in epoch time in [ns].
However, when converting from `DataPoints` to ROS messages, the timestamp needs to be split as the ROS messages don't support `int64` timestamps.
The timestamp is therefore split into two fields: `time_splitTime_high32` and `time__splitTime_low32`, representing the high and low 32 bits of the timestamp, respectively.
You won't typically see any differences between points when using the higher 32 bits of the timestamp, so use the lower 32 bits for any visualization purposes.
Moreover, `libpointmatcher_ros` also outputs an additional field called `elapsedTimeSec`, which represents the time elapsed between the first and the last point in the point cloud (with a lidar at 10 Hz, the values will lie between 0.0 to 0.1 seconds).
When converting back from ROS messages to `DataPoints`, the low and high fields get combined back into a single `int64` timestamp.

Below you can find a brief explication how the different timestamp types are treated when converted to `DataPoints` objects.

### Ouster-like
For Ouster-like timestamps, the timestamp is stored in the `uint32` field `t` in the ROS message.
The values represent the number of nanoseconds since the beginning of the scan, which is given in the `header.stamp` field.
To get a per-point timestamp, one has to add the per-point `t` value to the value obtained from the `header.stamp` field.

### Leishen-like
For Leishen-like timestamps, the timestamp is stored in the `float` field `time` in the ROS message.
The values represent the number of seconds (typically between 0.0 and 0.1) before the end of the scan, which is given in the `header.stamp` field.
To get a per-point timestamp, we have to subtract the per-point `time` value from the value obtained from the `header.stamp` field.

### Hesai-like
For Hesai-like timestamps, the timestamp is stored in the `double` field `timestamp` in the ROS message.
The values represent the number of seconds in epoch time.
This is the easest case as the timestamp is already in a 8-byte representation and we just need to multiply the per-point `timestamp` value by 1e9 to obtain nanoseconds and convert it to an `int64` datatype.

----------------
Note that we haven't tested the bridge with different lidars, which might need special treatment due to different timestamp representations.
To figure out how what exactly your ROS messages represent, you will need to investigate the driver code or the datasheet of the lidar.
