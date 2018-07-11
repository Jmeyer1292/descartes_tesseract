#include "descartes_tesseract/tesseract_state_adapter.h"
#include <urdf_parser/urdf_parser.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>

descartes_tesseract::TesseractStateAdapter::TesseractStateAdapter(const opw_kinematics::Parameters<double>& kin_params,
                                                                  const std::string& kin_base_frame,
                                                                  const std::string& kin_tool_frame)
  : kin_params_{kin_params}
  , kin_base_frame_(kin_base_frame)
  , kin_tool_frame_(kin_tool_frame)
{
}

bool descartes_tesseract::TesseractStateAdapter::initialize(const std::string& robot_description,
                                                            const std::string& group_name,
                                                            const std::string& world_frame,
                                                            const std::string& tcp_frame)
{
  // LOAD PARAMETERS //
  ros::NodeHandle nh; // We're going to load the model from the parameter server...
  std::string urdf_string, srdf_string;

  if (!nh.getParam(robot_description, urdf_string)) return false;
  if (!nh.getParam(robot_description + "_semantic", srdf_string)) return false;

  // BUILD URDF & SRDF MODELS //
  auto urdf_model = urdf::parseURDF(urdf_string);
  if (!urdf_model) return false;

  srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  if (!srdf_model->initString(*urdf_model, srdf_string)) return false;

  // TESSERACT //
  tesseract::tesseract_ros::KDLEnvPtr kdl_env =
      tesseract::tesseract_ros::KDLEnvPtr(new tesseract::tesseract_ros::KDLEnv);
  if (!kdl_env->init(urdf_model, srdf_model)) return false;

  collision_env_ptr_ = kdl_env;

  // Confirm that the manipulator group exists
  if (!collision_env_ptr_->hasManipulator(group_name))
  {
    return false;
  }
  manipulator_ = collision_env_ptr_->getManipulator(group_name);

  // Now we need to compute the "fixed" transform between the solver's base & tool frames and the
  // user-specified base & tool frames.
  // I'm going to set the joint state of the collision env to zero to ensure I have a valid
  // state for transform lookups later
  const auto joint_names = collision_env_ptr_->getJointNames();
  collision_env_ptr_->setState(joint_names, std::vector<double>(joint_names.size(), 0.0));

  const auto tf_world = collision_env_ptr_->getLinkTransform(world_frame);
  const auto tf_base = collision_env_ptr_->getLinkTransform(kin_base_frame_);
  const auto tf_tool0 = collision_env_ptr_->getLinkTransform(kin_tool_frame_);
  const auto tf_tip = collision_env_ptr_->getLinkTransform(tcp_frame);

  tool0_to_tip_ = descartes_core::Frame(tf_tool0.inverse() * tf_tip);
  world_to_base_ = descartes_core::Frame(tf_world.inverse() * tf_base);

  return true;
}

bool descartes_tesseract::TesseractStateAdapter::getIK(const Eigen::Affine3d& pose,
                                                       const std::vector<double>& seed_state,
                                                       std::vector<double>& joint_pose) const
{
  (void)pose;
  (void)seed_state;
  (void)joint_pose;
  return false;
}

static void printVector(const std::string& name, const std::vector<double>& v)
{
  std::stringstream ss;
  ss << name << ": [";
  for (auto&& a : v) ss << a << ", ";
  ss << "]";
  ROS_INFO_STREAM(ss.str());
}

bool descartes_tesseract::TesseractStateAdapter::getAllIK(const Eigen::Affine3d& pose,
                                                          std::vector<std::vector<double>>& joint_poses) const
{
//  ROS_WARN("new solve");
  joint_poses.clear();

  // Transform input pose
  Eigen::Affine3d tool_pose = world_to_base_.frame_inv * pose * tool0_to_tip_.frame;

  std::array<double, 6*8> sols;
  opw_kinematics::inverse(kin_params_, tool_pose, sols.data());

  // Check the output
  std::vector<double> tmp (6); // temporary storage for API reasons
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol);

      // TODO: make this better...
      std::copy(sol, sol + 6, tmp.data());
//      printVector("testing...", tmp);
      if (isValid(tmp))
      {
//        printVector("accepted!", tmp);

//        Eigen::Affine3d pose = opw_kinematics::forward(kin_params_, tmp.data());
//        ROS_INFO_STREAM("pose:\n" << pose.matrix());

        joint_poses.push_back(tmp);
      }
    }
  }

  return joint_poses.size() > 0;
}

bool descartes_tesseract::TesseractStateAdapter::getFK(const std::vector<double>& joint_pose,
                                                       Eigen::Affine3d& pose) const
{
  if (!isValid(joint_pose)) // TODO: Why is this a thing?
    return false;

  pose = opw_kinematics::forward<double>(kin_params_, joint_pose.data());
  pose = world_to_base_.frame * pose * tool0_to_tip_.frame_inv;
  return true;
}

int descartes_tesseract::TesseractStateAdapter::getDOF() const
{
  return int(manipulator_->numJoints());
}

bool descartes_tesseract::TesseractStateAdapter::isValid(const std::vector<double>& joint_pose) const
{
  // check collision
  return inLimits(joint_pose) && !isInCollision(joint_pose);
}

bool descartes_tesseract::TesseractStateAdapter::isValid(const Eigen::Affine3d& pose) const
{
  return true; // TODO: what?
}

bool descartes_tesseract::TesseractStateAdapter::isValidMove(const double* s, const double* f, double dt) const
{
  return false; // TODO: :(
}

std::vector<double> descartes_tesseract::TesseractStateAdapter::getJointVelocityLimits() const
{
  // TODO
  std::vector<double> limits (getDOF(), 1.0);
  return limits;
}

bool descartes_tesseract::TesseractStateAdapter::inLimits(const std::vector<double>& joints) const
{
  const auto& limits = manipulator_->getLimits();
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    if (joints[i] < limits(i, 0) || joints[i] > limits(i, 1)) return false;
  }

  return true;
}

bool descartes_tesseract::TesseractStateAdapter::isInCollision(const std::vector<double>& joints) const
{
  tesseract::ContactRequest req;
  req.link_names = manipulator_->getLinkNames();
  req.type = tesseract::ContactRequestType::SINGLE;
  auto fn = std::bind(&TesseractStateAdapter::isContactAllowed, this, std::placeholders::_1, std::placeholders::_2);
  req.isContactAllowed = fn;

  const auto& joint_names = manipulator_->getJointNames();
  Eigen::Map<const Eigen::VectorXd> jnts (joints.data(), joints.size());

  tesseract::ContactResultMap contact_map;
  collision_env_ptr_->calcCollisionsDiscrete(req, joint_names, jnts, contact_map);

  if (contact_map.size() > 0) {
    return true;
  }
  return false;
}

bool descartes_tesseract::TesseractStateAdapter::isContactAllowed(const std::string& a, const std::string& b) const
{
  return collision_env_ptr_->getAllowedCollisionMatrix()->isCollisionAllowed(a, b);
}
