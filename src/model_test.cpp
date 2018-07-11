#include "opw_kinematics/opw_parameters_examples.h"
#include "descartes_tesseract/tesseract_state_adapter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_test");
  ros::NodeHandle nh;

  const auto params = opw_kinematics::makeIrb2400_10<double>();

  // Construct model
  descartes_tesseract::TesseractStateAdapter model (params, "base_link", "tool0");

  // Initialize
  if (!model.initialize("robot_description", "manipulator", "base_link", "tool0"))
  {
    ROS_ERROR("Failed to initialize robot model");
    return 1;
  }

  ROS_INFO_STREAM("Robot has: " << model.getDOF() << " dofs");

  // Solve IK
  Eigen::Affine3d tp = Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, 0, 1.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
  std::vector<std::vector<double>> joint_poses;
  model.getAllIK(tp, joint_poses);

  ROS_INFO_STREAM("Joint sols: " << joint_poses.size());

  // WTF is using pluginlib? Getting the unloaded warning.
  return 0;
}
