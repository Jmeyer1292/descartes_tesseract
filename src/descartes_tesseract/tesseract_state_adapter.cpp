#include "descartes_tesseract/tesseract_state_adapter.h"

descartes_tesseract::TesseractStateAdapter::TesseractStateAdapter(const opw_kinematics::Parameters<double>& kin_params,
                                                                  const std::string& kin_base_frame,
                                                                  const std::string& kin_tool_frame)
{

}

bool descartes_tesseract::TesseractStateAdapter::initialize(const std::string& robot_description,
                                                            const std::string& group_name,
                                                            const std::string& world_frame,
                                                            const std::string& tcp_frame)
{

}

bool descartes_tesseract::TesseractStateAdapter::getIK(const Eigen::Affine3d& pose,
                                                       const std::vector<double>& seed_state,
                                                       std::vector<double>& joint_pose) const
{

}

bool descartes_tesseract::TesseractStateAdapter::getAllIK(const Eigen::Affine3d& pose,
                                                          std::vector<std::vector<double>>& joint_poses) const
{

}

bool descartes_tesseract::TesseractStateAdapter::getFK(const std::vector<double>& joint_pose,
                                                       Eigen::Affine3d& pose) const
{

}

int descartes_tesseract::TesseractStateAdapter::getDOF() const
{

}

bool descartes_tesseract::TesseractStateAdapter::isValid(const std::vector<double>& joint_pose) const
{

}

bool descartes_tesseract::TesseractStateAdapter::isValid(const Eigen::Affine3d& pose) const
{

}

bool descartes_tesseract::TesseractStateAdapter::isValidMove(const double* s, const double* f, double dt) const
{

}

std::vector<double> descartes_tesseract::TesseractStateAdapter::getJointVelocityLimits() const
{

}
