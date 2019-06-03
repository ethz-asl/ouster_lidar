#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cassert>

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

//<<<<<<< HEAD
//static PointOS1 nth_point(int ind, const uint8_t* col_buf) {
//    float h_angle_0 = col_h_angle(col_buf);
//    auto tte = trig_table[ind];
//    const uint8_t* px_buf = nth_px(ind, col_buf);
//    float r = px_range(px_buf) / 1000.0;
//    float h_angle = tte.beam_azimuth_angles + h_angle_0;
//
//    PointOS1 point;
//    point.reflectivity = px_reflectivity(px_buf);
//    point.intensity = px_signal_photons(px_buf);
//    point.x = -r * tte.cos_beam_altitude_angles * cosf(h_angle);
//    point.y = r * tte.cos_beam_altitude_angles * sinf(h_angle);
//    point.z = r * tte.sin_beam_altitude_angles;
//    point.ring = ind;
//
//    return point;
//}
//void add_packet_to_cloud(ns scan_start_ts, ns scan_duration,
//                         const PacketMsg& pm, CloudOS1& cloud) {
//    const uint8_t* buf = pm.buf.data();
//    for (int icol = 0; icol < columns_per_buffer; icol++) {
//        const uint8_t* col_buf = nth_col(icol, buf);
//        int32_t ts = col_timestamp(col_buf) - scan_start_ts.count();
//
//        for (int ipx = 0; ipx < pixels_per_column; ipx++) {
//            auto p = nth_point(ipx, col_buf);
//            p.time_offset_us = ts/1000;
//            cloud.push_back(p);
//        }
//    }
//}
//
//void spin(const client& cli,
//          const std::function<void(const PacketMsg& pm)>& lidar_handler,
//          const std::function<void(const PacketMsg& pm)>& imu_handler) {
//    PacketMsg lidar_packet, imu_packet;
//    lidar_packet.buf.resize(lidar_packet_bytes + 1);
//    imu_packet.buf.resize(imu_packet_bytes + 1);
//
//    while (ros::ok()) {
//        auto state = poll_client(cli);
//        if (state & ERROR) {
//            ROS_ERROR("spin: poll_client returned error");
//            return;
//        }
//        if (state & LIDAR_DATA) {
//            if (read_lidar_packet(cli, lidar_packet.buf.data()))
//                lidar_handler(lidar_packet);
//        }
//        if (state & IMU_DATA) {
//            if (read_imu_packet(cli, imu_packet.buf.data()))
//                imu_handler(imu_packet);
//        }
//        ros::spinOnce();
//    }
//}
//
//static ns nearest_scan_dur(ns scan_dur, ns ts) {
//    return ns((ts.count() / scan_dur.count()) * scan_dur.count());
//};
//=======
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
