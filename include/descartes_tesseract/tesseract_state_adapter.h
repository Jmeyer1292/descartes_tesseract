#ifndef DESCARTES_TESSERACT_STATE_ADAPTER_H
#define DESCARTES_TESSERACT_STATE_ADAPTER_H

#include <descartes_core/robot_model.h>
#include <opw_kinematics/opw_parameters.h>
#include <tesseract_ros/ros_basic_env.h>

namespace descartes_tesseract
{

class TesseractStateAdapter : public descartes_core::RobotModel
{
public:
  TesseractStateAdapter(const opw_kinematics::Parameters<double>& kin_params, const std::string& kin_base_frame,
                        const std::string& kin_tool_frame);

  bool initialize(const std::string& robot_description, const std::string& group_name, const std::string& world_frame,
                  const std::string& tcp_frame) override;

  bool getIK(const Eigen::Affine3d& pose, const std::vector<double>& seed_state,
             std::vector<double>& joint_pose) const override;

  bool getAllIK(const Eigen::Affine3d& pose, std::vector<std::vector<double> >& joint_poses) const override;

  bool getFK(const std::vector<double>& joint_pose, Eigen::Affine3d& pose) const override;

  int getDOF() const override;

  bool isValid(const std::vector<double>& joint_pose) const override;

  bool isValid(const Eigen::Affine3d& pose) const override;

  bool isValidMove(const double* s, const double* f, double dt) const override;

  std::vector<double> getJointVelocityLimits() const override;

private:
  tesseract::BasicEnvPtr collision_env_ptr_;
};

}

#endif
