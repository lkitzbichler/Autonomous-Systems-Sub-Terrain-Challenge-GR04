#include "mav_trajectory_generation/ros_conversions.h"

namespace mav_trajectory_generation
{

bool trajectoryToPolynomialTrajectoryMsg(
  const Trajectory & trajectory,
  mav_planning_msgs::msg::PolynomialTrajectory * msg)
{
  CHECK_NOTNULL(msg);
  msg->segments.clear();

  bool success = true;

  Segment::Vector segments;
  trajectory.getSegments(&segments);

  msg->segments.reserve(segments.size());
  for (size_t i = 0; i < segments.size(); ++i) {
    const Segment & segment = segments[i];

    if (segment.D() != 3 && segment.D() != 4 && segment.D() != 6) {
      LOG(ERROR) << "Dimension of position segment has to be 3, 4 or 6, but is "
                 << segment.D();
      success = false;
      break;
    }

    mav_planning_msgs::msg::PolynomialSegment segment_msg;
    mav_planning_msgs::EigenPolynomialSegment eigen_segment;
    eigen_segment.x = segment[0].getCoefficients();
    eigen_segment.y = segment[1].getCoefficients();
    eigen_segment.z = segment[2].getCoefficients();
    if (segment.D() == 4) {
      eigen_segment.yaw = segment[3].getCoefficients();
    } else if (segment.D() == 6) {
      eigen_segment.rx = segment[3].getCoefficients();
      eigen_segment.ry = segment[4].getCoefficients();
      eigen_segment.rz = segment[5].getCoefficients();
    }
    eigen_segment.num_coeffs = segment.N();
    eigen_segment.segment_time_ns = segment.getTimeNSec();

    mav_planning_msgs::polynomialSegmentMsgFromEigen(eigen_segment, &segment_msg);
    msg->segments.push_back(segment_msg);
  }

  if (!success) {
    msg->segments.clear();
  }
  return success;
}

// Converts a ROS2 polynomial trajectory msg into a Trajectory.
bool polynomialTrajectoryMsgToTrajectory(
  const mav_planning_msgs::msg::PolynomialTrajectory & msg,
  Trajectory * trajectory)
{
  mav_planning_msgs::EigenPolynomialTrajectory eigen_trajectory_msg;
  mav_planning_msgs::eigenPolynomialTrajectoryFromMsg(msg, &eigen_trajectory_msg);

  Segment::Vector segment_vector;
  for (const mav_planning_msgs::EigenPolynomialSegment & msg_segment : eigen_trajectory_msg) {
    int D = 3;
    if (msg_segment.yaw.size() > 0) {
      D = 4;
    }
    if (msg_segment.rx.size() > 0 && msg_segment.ry.size() > 0 && msg_segment.rz.size() > 0) {
      D = 6;
    }

    Segment segment(msg_segment.x.size(), D);
    segment[0].setCoefficients(msg_segment.x);
    segment[1].setCoefficients(msg_segment.y);
    segment[2].setCoefficients(msg_segment.z);
    if (D == 4) {
      segment[3].setCoefficients(msg_segment.yaw);
    } else if (D == 6) {
      segment[3].setCoefficients(msg_segment.rx);
      segment[4].setCoefficients(msg_segment.ry);
      segment[5].setCoefficients(msg_segment.rz);
    }
    segment.setTimeNSec(msg_segment.segment_time_ns);
    segment_vector.push_back(segment);
  }

  trajectory->setSegments(segment_vector);
  return true;
}

bool trajectoryToPolynomialTrajectoryMsg(
  const Trajectory & trajectory,
  mav_planning_msgs::msg::PolynomialTrajectory4D * msg)
{
  CHECK_NOTNULL(msg);
  msg->segments.clear();

  bool success = true;

  Segment::Vector segments;
  trajectory.getSegments(&segments);

  msg->segments.reserve(segments.size());
  for (size_t i = 0; i < segments.size(); ++i) {
    const Segment & segment = segments[i];

    if (segment.D() != 3 && segment.D() != 4) {
      LOG(ERROR) << "Dimension of position segment has to be 3 or 4, but is "
                 << segment.D();
      success = false;
      break;
    }

    mav_planning_msgs::msg::PolynomialSegment4D segment_msg;
    mav_planning_msgs::EigenPolynomialSegment eigen_segment;
    eigen_segment.x = segment[0].getCoefficients();
    eigen_segment.y = segment[1].getCoefficients();
    eigen_segment.z = segment[2].getCoefficients();
    if (segment.D() == 4) {
      eigen_segment.yaw = segment[3].getCoefficients();
    }
    eigen_segment.num_coeffs = segment.N();
    eigen_segment.segment_time_ns = segment.getTimeNSec();

    mav_planning_msgs::polynomialSegmentMsgFromEigen(eigen_segment, &segment_msg);
    msg->segments.push_back(segment_msg);
  }

  if (!success) {
    msg->segments.clear();
  }
  return success;
}

// Converts a ROS2 polynomial trajectory 4D msg into a Trajectory.
bool polynomialTrajectoryMsgToTrajectory(
  const mav_planning_msgs::msg::PolynomialTrajectory4D & msg,
  Trajectory * trajectory)
{
  mav_planning_msgs::EigenPolynomialTrajectory eigen_trajectory_msg;
  mav_planning_msgs::eigenPolynomialTrajectoryFromMsg(msg, &eigen_trajectory_msg);

  Segment::Vector segment_vector;
  for (const mav_planning_msgs::EigenPolynomialSegment & msg_segment : eigen_trajectory_msg) {
    int D = 3;
    if (msg_segment.yaw.size() > 0) {
      D = 4;
    }

    Segment segment(msg_segment.x.size(), D);
    segment[0].setCoefficients(msg_segment.x);
    segment[1].setCoefficients(msg_segment.y);
    segment[2].setCoefficients(msg_segment.z);
    if (D == 4) {
      segment[3].setCoefficients(msg_segment.yaw);
    }
    segment.setTimeNSec(msg_segment.segment_time_ns);
    segment_vector.push_back(segment);
  }

  trajectory->setSegments(segment_vector);
  return true;
}

}  // namespace mav_trajectory_generation
