#include <utility>
#include <boost/range/combine.hpp>
#include <moveit/robot_state/robot_state.h>
#include <relaxed_ik/objective.hpp>
#include <relaxed_ik/relaxed_ik_plugin.hpp>

namespace relaxed_ik {

static double groove_loss(double x, double t, double d, double c, double f, double g) {
    return -exp((-pow(x - t, d)) / (2.0 * pow(c, 2))) + f * pow(x - t, g);
}

double MatchEEPosiDoF::call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const Eigen::Vector3d linear = (goal.inverse() * current).translation();
    const double dist = linear[axis];
    return groove_loss(dist, 0, 2, 0.1, 10, 2);
}

double MatchEERotaDoF::call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const Eigen::Matrix3d rotation = (goal.inverse() * current).rotation();
    const Eigen::AngleAxisd aa(rotation);
    const double scaled_angle = std::abs(aa.angle() * aa.axis()[axis]);
    return groove_loss(scaled_angle, 0, 2, 0.1, 10, 2);
}

double MatchEEPosGoals::call(const std::vector<double> &joints, const relaxed_ik::Variables &v,
                             const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const double dist = (current.translation() - goal.translation()).norm();
    return groove_loss(dist, 0, 2, 0.1, 10, 2);
}

double MatchEEQuatGoals::call(const std::vector<double> &joints, const relaxed_ik::Variables &v,
                              const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const Eigen::Quaterniond current_orientation = Eigen::Quaterniond(current.rotation());
    const Eigen::Quaterniond goal_orientation = Eigen::Quaterniond(goal.rotation());
    const double dist = current_orientation.angularDistance(goal_orientation);
    return groove_loss(dist, 0, 2, 0.1, 10.0, 2);
}

ObjectiveMaster::ObjectiveMaster(const moveit::core::RobotModelConstPtr &m, Variables vars) : vars_(std::move(vars)) {
    state_ = std::make_shared<moveit::core::RobotState>(m);
    /*// Current RelaxedIK code
    objectives_.push_back(std::make_unique<MatchEEPosiDoF>(0)); weights_.push_back(50);
    objectives_.push_back(std::make_unique<MatchEEPosiDoF>(1)); weights_.push_back(50);
    objectives_.push_back(std::make_unique<MatchEEPosiDoF>(2)); weights_.push_back(50);
    objectives_.push_back(std::make_unique<MatchEERotaDoF>(0)); weights_.push_back(10);
    objectives_.push_back(std::make_unique<MatchEERotaDoF>(1)); weights_.push_back(10);
    objectives_.push_back(std::make_unique<MatchEERotaDoF>(2)); weights_.push_back(10);*/
    // RelaxedIK paper (this is faster)
    objectives_.push_back(std::make_unique<MatchEEPosGoals>()); weights_.push_back(50);
    objectives_.push_back(std::make_unique<MatchEEQuatGoals>()); weights_.push_back(40);
}

double ObjectiveMaster::call(const std::vector<double> &joints, std::vector<double> &grad) {
    state_->setJointGroupPositions(vars_.joint_group, joints);
    state_->updateLinkTransforms();
    double res = 0;
    for (const auto &[objective, weight] : boost::combine(objectives_, weights_)) {
        res += weight * objective->call(joints, vars_, *state_);
    }
    if (!grad.empty()) {
        for (size_t i = 0; i < joints.size(); ++i) {
            std::vector<double> x_h(joints);
            const double eps = 0.000000001;
            x_h[i] += eps;
            state_->setJointGroupPositions(vars_.joint_group, x_h);
            state_->updateLinkTransforms();
            double f_h = 0;
            for (const auto &[objective, weight] : boost::combine(objectives_, weights_)) {
                f_h += weight * objective->call(joints, vars_, *state_);
            }
            grad[i] = (-res + f_h) / eps;
        }
    }
    return res;
}
}