#include "opw_kinematics/opw_parameters_examples.h"
#include "descartes_tesseract/tesseract_state_adapter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_test");

  const auto params = opw_kinematics::makeIrb2400_10<double>();

  // Construct model
  descartes_tesseract::TesseractStateAdapter model (params, "base_link", "tool0");

  // Initialize
  if (!model.initialize("robot_description", "manipulator", "base_link", "tool0"))
  {
    ROS_ERROR("Failed to initialize robot model");
    return 1;
  }

  std::vector<std::vector<double>> joint_poses;
  model.getAllIK(Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, 0, 0.5), joint_poses);

  return 0;
}
