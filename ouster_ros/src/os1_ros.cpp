#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cassert>

#include <pcl/filters/frustum_culling.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster_ros/os1_ros.h"

namespace ouster_ros {
namespace OS1 {

using namespace ouster::OS1;

bool read_imu_packet(const client& cli, PacketMsg& m) {
    m.buf.resize(imu_packet_bytes + 1);
    return read_imu_packet(cli, m.buf.data());
}

bool read_lidar_packet(const client& cli, PacketMsg& m) {
    m.buf.resize(lidar_packet_bytes + 1);
    return read_lidar_packet(cli, m.buf.data());
}

sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& p,
                                   const std::string& frame) {
    const double standard_g = 9.80665;
    sensor_msgs::Imu m;
    const uint8_t* buf = p.buf.data();

    m.header.stamp.fromNSec(imu_gyro_ts(buf));
    m.header.frame_id = frame;

    m.orientation.x = 0;
    m.orientation.y = 0;
    m.orientation.z = 0;
    m.orientation.w = 0;

    m.linear_acceleration.x = imu_la_x(buf) * standard_g;
    m.linear_acceleration.y = imu_la_y(buf) * standard_g;
    m.linear_acceleration.z = imu_la_z(buf) * standard_g;

    m.angular_velocity.x = imu_av_x(buf) * M_PI / 180.0;
    m.angular_velocity.y = imu_av_y(buf) * M_PI / 180.0;
    m.angular_velocity.z = imu_av_z(buf) * M_PI / 180.0;

    for (int i = 0; i < 9; i++) {
        m.orientation_covariance[i] = -1;
        m.angular_velocity_covariance[i] = 0;
        m.linear_acceleration_covariance[i] = 0;
    }
    for (int i = 0; i < 9; i += 4) {
        m.linear_acceleration_covariance[i] = 0.01;
        m.angular_velocity_covariance[i] = 6e-4;
    }

    return m;
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(const CloudOS1& cloud, ns timestamp,
                                            const std::string& frame) {
    sensor_msgs::PointCloud2 msg{};
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp.fromNSec(timestamp.count());
    return msg;
}

sensor_msgs::PointCloud2 publishFovTrimmedCloud(float hfov, float vfov, Eigen::Matrix4f ousterPose, sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(
      new pcl::PointCloud<pcl::PointXYZ>);
  //Converting from ROS PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  //Note that converting to CloudOS1 causes linking error since precompiled pcl library doesn't know about CloudOS1 datatype.
  pcl::fromROSMsg(msg, *cloudPtr);
  //from http://docs.pointclouds.org/trunk/classpcl_1_1_frustum_culling.html
  pcl::FrustumCulling<pcl::PointXYZ> fc; 
  fc.setInputCloud(cloudPtr);
  fc.setVerticalFOV(vfov);
  fc.setHorizontalFOV(hfov);
  //fc.setHorizontalFOV(120);
  
  fc.setNearPlaneDistance(0.01);
  fc.setFarPlaneDistance(100);
  Eigen::Matrix4f ouster2frustum; // this thing assusms pose x-forward, y-up, z right. rotate
                              // -90 along x-axis to align with x-forward, y-left, z-up.
  ouster2frustum << -1, 0, 0, 0, 
                 0, -1, 0, 0, 
                 0, 0, 1, 0, 
                 0, 0, 0, 1;
  // init2lidar <<  1, 0, 0, 0,
  //                0, 0, -1, 0,
  //                0, 1, 0, 0,
  //                0, 0, 0, 1;
  Eigen::Matrix4f pose_new = ousterPose * ouster2frustum;
  fc.setCameraPose(pose_new);
  pcl::PointCloud<pcl::PointXYZ> filtered_pc;
  fc.filter(filtered_pc);
  sensor_msgs::PointCloud2 ret;
  pcl::toROSMsg(filtered_pc, ret);
  ret.header.frame_id = msg.header.frame_id;
  ret.header = msg.header;
  return ret;
}

geometry_msgs::TransformStamped transform_to_tf_msg(
    const std::vector<double>& mat, const std::string& frame,
    const std::string& child_frame) {
    assert(mat.size() == 16);

    tf2::Transform tf{};

    tf.setOrigin({mat[3] / 1e3, mat[7] / 1e3, mat[11] / 1e3});
    tf.setBasis({mat[0], mat[1], mat[2], mat[4], mat[5], mat[6], mat[8], mat[9],
                 mat[10]});

    geometry_msgs::TransformStamped msg{};
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame;
    msg.child_frame_id = child_frame;
    msg.transform = tf2::toMsg(tf);

    return msg;
}
}
}
