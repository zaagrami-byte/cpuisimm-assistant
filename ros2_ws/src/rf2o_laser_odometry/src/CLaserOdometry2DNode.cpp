/** ****************************************************************************************
*  RF2O — Planar Odometry from a Radial Laser Scanner (ICRA'16).
*  Maintainer: Javier G. Monroy (MAPIR group) — Modifications: Jeremie Deray
*  ISIMM : namespace fermé proprement, quaternion valide, QoS capteur, TF laser obligatoire
*  avant init, logs throttlés.
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.h"

namespace rf2o {

bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  geometry_msgs::msg::TransformStamped tf_laser;
  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[rf2o] TF %s -> %s indisponible : %s",
                         base_frame_id.c_str(), last_scan.header.frame_id.c_str(), ex.what());
    return false;
  }

  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;
  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);
  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  rf2o_ref.setLaserPose(laser_tf);
  return true;
}

void CLaserOdometry2DNode::sanitizeInitialPose()
{
  auto &q = initial_robot_pose.pose.pose.orientation;
  const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (!std::isfinite(n) || n < 1e-6)
  {
    RCLCPP_WARN(get_logger(), "[rf2o] Quaternion initial invalide -> identité");
    q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
  }
  else
  {
    q.x /= n; q.y /= n; q.z /= n; q.w /= n;
  }
}

bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}

void CLaserOdometry2DNode::process()
{
  if (!rf2o_ref.is_initialized())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "[rf2o] En attente du premier scan et du TF %s -> laser",
                         base_frame_id.c_str());
    return;
  }
  if (!scan_available())
    return;

  new_scan_available = false;   // évite de traiter deux fois le même scan
  if (rf2o_ref.odometryCalculation(last_scan))
    publish();
  else
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[rf2o] Calcul d'odométrie échoué, pose non publiée");
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (!GT_pose_initialized)
    return;

  last_scan = *new_scan;
  rf2o_ref.current_scan_time = last_scan.header.stamp;

  if (!rf2o_ref.first_laser_scan)
  {
    if (new_scan->ranges.size() != rf2o_ref.width)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[rf2o] Taille de scan changée (%zu != %u) : scan ignoré",
                           new_scan->ranges.size(), rf2o_ref.width);
      return;
    }
    for (unsigned int i = 0; i < rf2o_ref.width; i++)
      rf2o_ref.range_wf(i) = new_scan->ranges[i];
    new_scan_available = true;
  }
  else
  {
    // Le TF base_link -> laser doit être connu avant d'initialiser (sinon offset faux).
    if (!setLaserPoseFromTf())
      return;
    sanitizeInitialPose();
    rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
    rf2o_ref.first_laser_scan = false;
  }
}

void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}

void CLaserOdometry2DNode::publish()
{
  RCLCPP_DEBUG(get_logger(), "[rf2o] Publishing Odom Topic");
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  nav_msgs::msg::Odometry odom;

  odom.header.stamp = rf2o_ref.last_odom_time;
  odom.header.frame_id = odom_frame_id;
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;
  odom_pub->publish(odom);

  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "[rf2o] Publishing TF: [odom] to [base_link]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    odom_broadcaster->sendTransform(odom_trans);
  }
}

} /* namespace rf2o */

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();
  rclcpp::Rate rate(myLaserOdomNode->freq);
  while (rclcpp::ok())
  {
    myLaserOdomNode->process();
    rclcpp::spin_some(myLaserOdomNode);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
