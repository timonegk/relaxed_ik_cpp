#include <utility>
#include <boost/range/combine.hpp>
#include <moveit/robot_state/robot_state.h>
#include <relaxed_ik/objective.hpp>
#include <relaxed_ik/relaxed_ik_plugin.hpp>

namespace relaxed_ik {

static double groove_loss(double x, double t, double d, double c, double f, double g) {
    return -exp((-pow(x - t, d)) / (2.0 * pow(c, 2))) + f * pow(x - t, g);
}

static double swamp_loss(double x, double l_bound, double u_bound, double f1, double f2, double p1) {
    double x_scaled = (2 * x - l_bound - u_bound) / (u_bound - l_bound);
    double b = std::pow((-1 / std::log(0.05)), 1 / p1);
    return (f1 + f2 * std::pow(x_scaled, 2)) * (1 - std::exp(-std::pow(x/b, p1))) - 1.0;
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

SelfCollision::SelfCollision(const moveit::core::RobotModelConstPtr &robot_model) {
    const std::vector<const moveit::core::LinkModel*>& links = robot_model->getLinkModelsWithCollisionGeometry();
    geoms_.resize(robot_model->getLinkGeometryCount());
    for (std::size_t i = 0 ; i < links.size() ; ++i) {
        for (std::size_t j = 0; j < links[i]->getShapes().size(); ++j) {
            collision_detection::FCLGeometryConstPtr g = collision_detection::createCollisionGeometry(links[i]->getShapes()[j],
                                                                                                      1, 0.0, links[i], j);
            if (g) {
                geoms_.push_back(g);
                fcl_objs_[links[i]->getName()].push_back(std::make_shared<fcl::CollisionObjectd>(g->collision_geometry_));
            } else {
                std::cerr << "Unable to construct collision geometry for link '" << links[i]->getName() << "'"
                          << std::endl;
            }
        }
    }
    acm_ = std::make_shared<const collision_detection::AllowedCollisionMatrix>(*robot_model->getSRDF());
}
double SelfCollision::call(const std::vector<double> &joints, const relaxed_ik::Variables &v,
                           const moveit::core::RobotState &state) {
    for (const auto &[link_name, collision_objects] : fcl_objs_) {
        const auto link_model = state.getLinkModel(link_name);
        const auto &link_transform = state.getGlobalLinkTransform(link_name);
        const auto link_collision_offset = link_model->getCollisionOriginTransforms();
        for (std::size_t i = 0; i < collision_objects.size(); ++i) {
            collision_objects[i]->setTransform(link_transform * link_collision_offset[i]);
            collision_objects[i]->computeAABB();
        }
    }

    double res = 0;
    collision_detection::AllowedCollision::Type col_type;
    for (const auto &[l1, c1] : fcl_objs_) {
        for (const auto &[l2, c2] : fcl_objs_) {
            // make sure to only do collision detection once per pair
            if (l1 < l2) {
                double min_distance = DBL_MAX;
                if (acm_->getEntry(l1, l2, col_type) && col_type == collision_detection::AllowedCollision::Type::ALWAYS) {
                    // if the links are in the ACM and the collision is allowed, skip them
                    continue;
                } else {
                    // if the collision is not allowed, do the distance check
                    // these two loops will normally only have a single element
                    for (const auto &o1: c1) {
                        for (const auto &o2: c2) {
                            fcl::DistanceResultd unused;
                            auto start = std::chrono::high_resolution_clock::now();
                            double distance = fcl::distance(o1.get(), o2.get(), fcl::DistanceRequestd(false),
                                                            unused);

                            if (distance < min_distance) {
                                min_distance = distance;
                            }
                        }
                    }
                    // change from original RangedIK: l_bound is 0.01 instead of 0.02
                    res += swamp_loss(min_distance, 0.01, 1.5, 60.0, 0.0001, 30);
                }
            }
        }
    }
    return res;
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
    objectives_.push_back(std::make_unique<SelfCollision>(m)); weights_.push_back(1);
}

double ObjectiveMaster::call(const std::vector<double> &joints, std::vector<double> &grad) {
    state_->setJointGroupPositions(vars_.joint_group, joints);
    state_->updateLinkTransforms();
    double res = 0;
    for (const auto &[objective, weight] : boost::combine(objectives_, weights_)) {
        res += weight * objective->call(joints, vars_, *state_);
    }
    if (!grad.empty()) {
        for (std::size_t i = 0; i < joints.size(); ++i) {
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
