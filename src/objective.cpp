#include <utility>
#include <moveit/robot_state/robot_state.h>
#include <relaxed_ik/objective.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace relaxed_ik {

static double groove_loss(double x, double t, double d, double c, double f, double g) {
    // -e^{-\frac{(x-t)^{d}}{2\cdot c^{2}}}+f\cdot (x-t)^{g}
    return -exp((-pow(x - t, d)) / (2.0 * pow(c, 2))) + f * pow(x - t, g);
}

static double swamp_loss(double x, double l_bound, double u_bound, double f1, double f2, double p1) {
    double x_scaled = (2 * x - l_bound - u_bound) / (u_bound - l_bound);
    double b = std::pow((-1 / std::log(0.05)), 1 / p1);
    return (f1 + f2 * std::pow(x_scaled, 2)) * (1 - std::exp(-std::pow(x/b, p1))) - 1.0;
}

double MatchEEPosiDoF::call(const std::vector<double> &, const Variables &v, const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const Eigen::Vector3d linear = (goal.inverse() * current).translation();
    const double dist = linear[axis];
    return groove_loss(dist, 0, 2, 0.1, 10, 2);
}

double MatchEERotaDoF::call(const std::vector<double> &, const Variables &v, const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const Eigen::Matrix3d rotation = (goal.inverse() * current).rotation();
    const Eigen::AngleAxisd aa(rotation);
    const double scaled_angle = std::abs(aa.angle() * aa.axis()[axis]);
    return groove_loss(scaled_angle, 0, 2, 0.1, 10, 2);
}

double MatchEEPosGoals::call(const std::vector<double> &, const relaxed_ik::Variables &v,
                             const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const double dist = (current.translation() - goal.translation()).norm();
    return groove_loss(dist, 0, 2, 0.1, 10, 2);
}

double MatchEEQuatGoals::call(const std::vector<double> &, const relaxed_ik::Variables &v,
                              const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d goal = v.target_pose;
    const Eigen::Quaterniond current_orientation = Eigen::Quaterniond(current.rotation());
    const Eigen::Quaterniond goal_orientation = Eigen::Quaterniond(goal.rotation());
    const double dist = current_orientation.angularDistance(goal_orientation);
    return groove_loss(dist, 0, 2, 0.1, 10.0, 2);
}

double IKCostFnGoal::call(const std::vector<double> &joints, const relaxed_ik::Variables &v,
                           const moveit::core::RobotState &state) {
	return fn_(pose_, state, state.getRobotModel()->getJointModelGroup(v.joint_group), v.seed_state);
}

ObjectiveMaster::ObjectiveMaster(const moveit::core::RobotModelConstPtr &m, Variables vars, const std::vector<std::pair<std::shared_ptr<Objective>, double>> &objectives) : vars_(std::move(vars)) {
    state_ = std::make_shared<moveit::core::RobotState>(m);
    objectives_ = objectives;
    // RelaxedIK code uses MatchEEPosiDoF for each axis with weight 50 and Rota for each axis with weight 10
    // The paper uses MatchEEPosGoals and MatchEEQuatGoals with weights 1 each. This is faster.
}

double ObjectiveMaster::call(const std::vector<double> &joints, std::vector<double> &grad) {
    state_->setJointGroupPositions(vars_.joint_group, joints);
    state_->update();
    double res = 0;
    for (const auto &[objective, weight] : objectives_) {
        res += weight * objective->call(joints, vars_, *state_);
    }
    if (!grad.empty()) {
        for (std::size_t i = 0; i < joints.size(); ++i) {
            std::vector<double> x_h(joints);
            const double eps = 0.0000001;
            x_h[i] += eps;
            state_->setJointGroupPositions(vars_.joint_group, x_h);
            state_->update();
            double f_h = 0;
            for (const auto &[objective, weight] : objectives_) {
                f_h += weight * objective->call(joints, vars_, *state_);
            }
            grad[i] = (-res + f_h) / eps;
        }
    }
    return res;
}
}
