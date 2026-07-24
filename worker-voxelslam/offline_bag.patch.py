#!/usr/bin/env python3
"""Patch hku-mars/Voxel-SLAM to read its input directly from a rosbag inside
the node (offline, synchronous) instead of via ROS subscribe callbacks driven
by an external publisher.

Why: our Python replay bridge cannot sustain the real 10Hz publish rate, so it
lagged ~46s behind, leaving a large subscriber-queue tail that sent the node
into a Reset loop at finish -- and it forced coarser header-interval timing
(point_notime=1) to avoid a "LiDAR time regress" abort. Reading the bag
directly inside thd_odometry_localmapping removes the pacing problem entirely
(the node consumes messages at its own processing speed), lets us keep true
per-point offset_time deskew (point_notime=0), and mirrors how the vendor's
own offline engine (share_slam2_offline.exe) actually runs.

Also folds in the S20-specific IMU unit fix: this device's raw accelerometer
is in g, not m/s^2 (the same fix our bridges applied), so imu_handler scales
linear_acceleration by 9.80665 before buffering.

Applied at image-build time against the freshly-cloned sources.
"""
import io
import re
import sys

HPP = "/root/catkin_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.hpp"
CPP = "/root/catkin_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.cpp"
CMK = "/root/catkin_ws/src/Voxel-SLAM/VoxelSLAM/CMakeLists.txt"
VMH = "/root/catkin_ws/src/Voxel-SLAM/VoxelSLAM/src/voxel_map.hpp"


def patch_voxelmap(text: str) -> str:
    # Carry per-point LiDAR intensity (reflectivity) through the pipeline so it
    # can be written into the saved PCD -- pointVar otherwise drops it, leaving
    # the exported cloud with no intensity channel for visualization.
    anchor = "  Eigen::Vector3d pnt;\n  Eigen::Matrix3d var;\n};"
    assert anchor in text, "pointVar struct anchor not found"
    return text.replace(
        anchor,
        "  Eigen::Vector3d pnt;\n  Eigen::Matrix3d var;\n  float inten = 0;\n};",
        1,
    )


def patch_intensity_cpp(text: str) -> str:
    # (a) point_notime deskew branch
    a1 = "        pv.pnt << ap.x, ap.y, ap.z;\n        pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;"
    assert a1 in text, "intensity anchor a1 not found"
    text = text.replace(
        a1,
        "        pv.pnt << ap.x, ap.y, ap.z; pv.inten = ap.intensity;\n"
        "        pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;",
        1,
    )
    # (b) true-deskew branch (the one we use, point_notime=0)
    a2 = "        pv.pnt = P_compensate;\n        pvec.push_back(pv);"
    assert a2 in text, "intensity anchor a2 not found"
    text = text.replace(
        a2,
        "        pv.pnt = P_compensate; pv.inten = it_pcl->intensity;\n"
        "        pvec.push_back(pv);",
        1,
    )
    # (c) write intensity into the saved PCD
    a3 = "      ap.x = pw.pnt[0]; ap.y = pw.pnt[1]; ap.z = pw.pnt[2];\n      pl_save.push_back(ap);"
    assert a3 in text, "intensity anchor a3 not found"
    text = text.replace(
        a3,
        "      ap.x = pw.pnt[0]; ap.y = pw.pnt[1]; ap.z = pw.pnt[2]; ap.intensity = pw.inten;\n"
        "      pl_save.push_back(ap);",
        1,
    )
    # (d) relax the initialization degeneracy threshold. On narrow structures
    # (e.g. the e6b4bbe7 bridge) plane normals are mostly parallel, so the
    # smallest eigenvalue of the normal-covariance stays below the stock 15
    # threshold, marking init degenerate forever -> endless Reset loop. S20
    # scans routinely include such narrow corridors/bridges, so lower it to 8
    # (still rejects truly featureless init, but accepts constrained geometry).
    a4 = "is_degrade = eigvalue[0] < 15 ? true : false;"
    assert a4 in text, "init degrade threshold anchor not found"
    text = text.replace(a4, "is_degrade = eigvalue[0] < 8 ? true : false;", 1)
    return text


def patch_cmake(text: str) -> str:
    # The stock find_package(catkin COMPONENTS ...) omits rosbag, but the
    # offline reader links rosbag::Bag/View. Add it.
    anchor = "  livox_ros_driver\n)"
    assert anchor in text, "CMake catkin COMPONENTS anchor not found"
    return text.replace(anchor, "  livox_ros_driver\n  rosbag\n)", 1)


def patch_hpp(text: str) -> str:
    # 1) includes for rosbag + livox CustomMsg (feature_point.hpp already pulls
    #    livox_ros_driver, but be explicit for the bag View instantiate call).
    text = text.replace(
        '#include "BTC.h"',
        '#include "BTC.h"\n'
        '#include <rosbag/bag.h>\n'
        '#include <rosbag/view.h>\n'
        '#include <livox_ros_driver/CustomMsg.h>',
        1,
    )

    # 2) IMU g -> m/s^2 (S20 built-in IMU reports g). Insert right after the
    #    per-message copy, before it is buffered.
    anchor = "  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));"
    inject = (
        anchor
        + "\n\n"
        + "  // S20 built-in MID-360 IMU reports linear_acceleration in g, not\n"
        + "  // m/s^2 (see slamcloude worker bridges); Voxel-SLAM/ROS expect m/s^2.\n"
        + "  msg->linear_acceleration.x *= 9.80665;\n"
        + "  msg->linear_acceleration.y *= 9.80665;\n"
        + "  msg->linear_acceleration.z *= 9.80665;"
    )
    assert anchor in text, "imu_handler anchor not found"
    text = text.replace(anchor, inject, 1)

    # 3) Offline bag reader globals + feed function, added just before
    #    sync_packages so it sees imu_buf/pcl_buf/mBuf and the handlers.
    reader = r'''
// ---------------- offline rosbag reader (slamcloude S20) ----------------
// Replaces the real-time ROS subscribe path: the node pulls messages straight
// out of the bag at its own speed, so there is no publish-rate dependency and
// no subscriber-queue backlog.
static rosbag::Bag  g_bag;
static rosbag::View *g_view = nullptr;
static rosbag::View::iterator g_it;
static bool   g_bag_open = false;
static bool   g_bag_done = false;
static string g_lid_topic_off, g_imu_topic_off;

inline void offline_bag_open(const string &bagpath, const string &lid_topic, const string &imu_topic)
{
  g_lid_topic_off = lid_topic;
  g_imu_topic_off = imu_topic;
  g_bag.open(bagpath, rosbag::bagmode::Read);
  vector<string> topics; topics.push_back(lid_topic); topics.push_back(imu_topic);
  g_view = new rosbag::View(g_bag, rosbag::TopicQuery(topics));
  g_it = g_view->begin();
  g_bag_open = true;
  printf("[offline] opened bag %s (lid=%s imu=%s)\n", bagpath.c_str(), lid_topic.c_str(), imu_topic.c_str());
}

// Feed up to max_msgs messages from the bag into the same buffers the ROS
// callbacks would fill. Returns false once the bag is exhausted.
inline bool offline_bag_feed(int max_msgs)
{
  if(!g_bag_open) return false;
  int n = 0;
  while(n < max_msgs && g_it != g_view->end())
  {
    const rosbag::MessageInstance &m = *g_it;
    // Use the bag RECORD time (envelope), not the message's embedded
    // header.stamp: the S20's Livox CustomMsg carries the sensor's own
    // free-running/unsynced clock (~1.5y off), while RTK/camera and the rest
    // of the pipeline are timestamped on the bag clock. Overwriting the stamp
    // here keeps the SLAM trajectory on the same clock as the RTK fixes used
    // downstream for georeferencing.
    ros::Time bagt = m.getTime();
    if(m.getTopic() == g_lid_topic_off)
    {
      livox_ros_driver::CustomMsg::ConstPtr lm = m.instantiate<livox_ros_driver::CustomMsg>();
      if(lm)
      {
        livox_ros_driver::CustomMsg::Ptr lm2(new livox_ros_driver::CustomMsg(*lm));
        lm2->header.stamp = bagt;
        // Clamp the intra-scan offset_time span below the scan interval. Some
        // S20 bags (e.g. e6b4bbe7) have offset_time covering the full ~100ms
        // scan while header intervals jitter down to ~82ms, so pcl_end_time
        // overruns the next scan's begin and trips ekf_imu.hpp's
        // "LiDAR time regress" exit(0). Linearly scaling each scan's span to
        // <=60ms (safely below the min interval seen on all S20 bags) keeps
        // relative point timing/ordering while staying regress-safe.
        uint32_t maxoff = 0;
        for(const auto &pt : lm2->points)
          if(pt.offset_time > maxoff) maxoff = pt.offset_time;
        const uint32_t OFFCAP = 60000000u;  // 60ms in ns
        if(maxoff > OFFCAP)
        {
          double sc = (double)OFFCAP / (double)maxoff;
          for(auto &pt : lm2->points)
            pt.offset_time = (uint32_t)(pt.offset_time * sc);
        }
        // pcl_handler takes a non-const T& (template), so pass a named lvalue.
        livox_ros_driver::CustomMsg::ConstPtr lm2c(lm2);
        pcl_handler(lm2c);
      }
    }
    else if(m.getTopic() == g_imu_topic_off)
    {
      sensor_msgs::Imu::ConstPtr im = m.instantiate<sensor_msgs::Imu>();
      if(im)
      {
        sensor_msgs::Imu::Ptr im2(new sensor_msgs::Imu(*im));
        im2->header.stamp = bagt;
        imu_handler(sensor_msgs::Imu::ConstPtr(im2));
      }
    }
    ++g_it; ++n;
  }
  if(g_it == g_view->end()) g_bag_done = true;
  return !g_bag_done;
}
'''
    marker = "bool sync_packages("
    assert marker in text, "sync_packages marker not found"
    text = text.replace(marker, reader + "\n" + marker, 1)
    return text


def patch_cpp(text: str) -> str:
    # In the odometry loop, replace the real-time spin with an offline bag feed
    # and auto-finish once the bag is drained. `ros::spinOnce()` still runs so
    # the loop-closure/GBA threads' publishers stay serviced, but data now
    # comes from the bag, not subscribers.
    old = "      ros::spinOnce();\n      if(loop_detect == 1)"
    new = (
        "      ros::spinOnce();\n"
        "      offline_bag_feed(200);\n"
        "      if(g_bag_done && pcl_buf.empty())\n"
        "        n.setParam(\"finish\", true);\n"
        "      if(loop_detect == 1)"
    )
    assert old in text, "odometry-loop spinOnce anchor not found"
    text = text.replace(old, new, 1)

    # In main(), open the bag (path from ~bag_path param) instead of relying on
    # an external publisher. Insert right after the VOXEL_SLAM object is built
    # so its General/lid_topic & General/imu_topic params are already loaded.
    anchor = "  VOXEL_SLAM vs(n);"
    inject = (
        anchor
        + "\n\n"
        + "  {\n"
        + "    std::string bagpath, lid_topic, imu_topic;\n"
        + "    n.param<std::string>(\"bag_path\", bagpath, \"\");\n"
        + "    n.param<std::string>(\"General/lid_topic\", lid_topic, \"/livox/lidar\");\n"
        + "    n.param<std::string>(\"General/imu_topic\", imu_topic, \"/livox/imu\");\n"
        + "    if(!bagpath.empty()) offline_bag_open(bagpath, lid_topic, imu_topic);\n"
        + "  }"
    )
    assert anchor in text, "main VOXEL_SLAM anchor not found"
    text = text.replace(anchor, inject, 1)
    return text


def main():
    with io.open(HPP, encoding="utf-8") as f:
        hpp = f.read()
    with io.open(CPP, encoding="utf-8") as f:
        cpp = f.read()
    with io.open(CMK, encoding="utf-8") as f:
        cmk = f.read()
    with io.open(VMH, encoding="utf-8") as f:
        vmh = f.read()
    hpp2 = patch_hpp(hpp)
    cpp2 = patch_intensity_cpp(patch_cpp(cpp))
    cmk2 = patch_cmake(cmk)
    vmh2 = patch_voxelmap(vmh)
    with io.open(HPP, "w", encoding="utf-8") as f:
        f.write(hpp2)
    with io.open(CPP, "w", encoding="utf-8") as f:
        f.write(cpp2)
    with io.open(CMK, "w", encoding="utf-8") as f:
        f.write(cmk2)
    with io.open(VMH, "w", encoding="utf-8") as f:
        f.write(vmh2)
    print("offline_bag + intensity patch applied to voxelslam.hpp/.cpp, voxel_map.hpp, CMakeLists.txt")


if __name__ == "__main__":
    main()
