/**
 * @file
 * @brief Example node to publish OS-1 point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>
#include <pcl/filters/frustum_culling.h>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PacketMsg = ouster_ros::PacketMsg;
using CloudOS1 = ouster_ros::OS1::CloudOS1;
using PointOS1 = ouster_ros::OS1::PointOS1;

namespace OS1 = ouster::OS1;
using namespace std;
int main(int argc, char** argv) {
  ros::init(argc, argv, "os1_cloud_node");
  ros::NodeHandle nh("~");

  auto tf_prefix = nh.param("tf_prefix", std::string{});
  auto sensor_frame = tf_prefix + "/os1_sensor";
  auto imu_frame = tf_prefix + "/os1_imu";
  auto lidar_frame = tf_prefix + "/os1_lidar";

  ouster_ros::OS1ConfigSrv cfg{};
  auto client = nh.serviceClient<ouster_ros::OS1ConfigSrv>("os1_config");
  client.waitForExistence();
  if (!client.call(cfg)) {
    ROS_ERROR("Calling os1 config service failed");
    return EXIT_FAILURE;
  }

  uint32_t H = OS1::pixels_per_column;
  uint32_t W = OS1::n_cols_of_lidar_mode(
      OS1::lidar_mode_of_string(cfg.response.lidar_mode));
  auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
  auto lidar_pub2 = nh.advertise<sensor_msgs::PointCloud2>("points2", 10);
  auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

  auto xyz_lut = OS1::make_xyz_lut(W, H, cfg.response.beam_azimuth_angles,
                                   cfg.response.beam_altitude_angles);

  CloudOS1 cloud{W, H};
  auto it = cloud.begin();
  sensor_msgs::PointCloud2 msg{};

  auto batch_and_publish = OS1::batch_to_iter<CloudOS1::iterator>(
      xyz_lut, W, H, {}, &PointOS1::make,
      [&](uint64_t scan_ts) mutable {
        msg = ouster_ros::OS1::cloud_to_cloud_msg(
            cloud, std::chrono::nanoseconds{scan_ts}, lidar_frame);
        lidar_pub.publish(msg);

        #if 1
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(msg, *cloudPtr);
        pcl::FrustumCulling<pcl::PointXYZ> fc;
        fc.setInputCloud(cloudPtr);
        fc.setVerticalFOV(120);
        fc.setHorizontalFOV(120);
        fc.setNearPlaneDistance(0.01);
        fc.setFarPlaneDistance(100);
        Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f init2lidar,cam2lidar; // this thing assusms pose x-forward, y-up, z right. rotate -90 along x-axis to align with x-forward, y-left, z-up.
        // init2lidar << 1, 0, 0, 0,
        //        0, 0, 1, 0,
        //        0, -1, 0, 0,
        //        0, 0, 0, 1;
        //not perfectly work
        init2lidar << -1, 0, 0, 0,
               0, -1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
        // init2lidar <<  1, 0, 0, 0,
        //                0, 0, -1, 0,
        //                0, 1, 0, 0,
        //                0, 0, 0, 1;
        Eigen::Matrix4f pose_new = init_pose*init2lidar;

        fc.setCameraPose(pose_new);
        
        pcl::PointCloud<pcl::PointXYZ> target;
        fc.filter(target);
        sensor_msgs::PointCloud2 msg2;
        pcl::toROSMsg(target, msg2);
        msg2.header.frame_id = msg.header.frame_id;
        msg2.header = msg.header;
        lidar_pub2.publish(msg2);
        #endif


      });

  auto lidar_handler = [&](const PacketMsg& pm) mutable {
    batch_and_publish(pm.buf.data(), it);
  };

  auto imu_handler = [&](const PacketMsg& p) {
    imu_pub.publish(ouster_ros::OS1::packet_to_imu_msg(p, imu_frame));
  };

  auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
      "lidar_packets", 2048, lidar_handler);
  auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
      "imu_packets", 100, imu_handler);

  // publish transforms
  tf2_ros::StaticTransformBroadcaster tf_bcast{};

  tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
      cfg.response.imu_to_sensor_transform, sensor_frame, imu_frame));

  tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
      cfg.response.lidar_to_sensor_transform, sensor_frame, lidar_frame));

  ros::spin();

  return EXIT_SUCCESS;
}
