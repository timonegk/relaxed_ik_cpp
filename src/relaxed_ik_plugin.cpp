#include <nlopt.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <relaxed_ik/relaxed_ik_plugin.hpp>

namespace relaxed_ik {
    bool RelaxedIKPlugin::initialize(const rclcpp::Node::SharedPtr &node,
                                     const moveit::core::RobotModel &robot_model,
                                     const std::string &group_name,
                                     const std::string &base_frame,
                                     const std::vector<std::string> &tip_frames,
                                     double search_discretization) {
        node_ = node;
        storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
        parameter_listener_ = std::make_shared<ParamListener>(
                node_,
                std::string("robot_description_kinematics.").append(group_name));
        state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        vars_.joint_group = group_name_;
        jmg_ = robot_model_->getJointModelGroup(group_name_);
        if (tip_frames.size() > 1) {
            RCLCPP_ERROR(node_->get_logger(), "RelaxedIK does not support multiple tip frames");
            return false;
        }
        vars_.ee_name = tip_frames[0];
        joint_names_ = std::vector<std::string>(jmg_->getJointModelNames());
        link_names_ = std::vector<std::string>(jmg_->getLinkModelNames());
        return true;
    }

    double RelaxedIKPlugin::wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
        return reinterpret_cast<ObjectiveMaster *>(data)->call(x, grad);
    }

    bool RelaxedIKPlugin::searchPositionIK(
            std::vector<geometry_msgs::msg::Pose> const &ik_poses,
            std::vector<double> const &ik_seed_state,
            double timeout,
            std::vector<double> const &,
            std::vector<double> &solution,
            IKCallbackFn const &solution_callback,
            IKCostFn const &cost_function,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            kinematics::KinematicsQueryOptions const &options,
            moveit::core::RobotState const *context_state) const {
        (void) context_state;  // not used
        const auto start_time = std::chrono::high_resolution_clock::now();
        const auto params = parameter_listener_->get_params();

        const std::size_t n_joints = jmg_->getVariableCount();
        auto opt = nlopt::opt(nlopt::LD_SLSQP, n_joints);
        std::vector<double> lower_bounds, upper_bounds;
        for (const auto &variable: jmg_->getVariableNames()) {
            auto bounds = robot_model_->getVariableBounds(variable);
            lower_bounds.push_back(bounds.min_position_);
            upper_bounds.push_back(bounds.max_position_);
        }
        opt.set_lower_bounds(lower_bounds);
        opt.set_upper_bounds(upper_bounds);

        if (ik_poses.size() > 1) {
            RCLCPP_ERROR(node_->get_logger(), "RelaxedIK does not support multiple tip frames");
            return false;
        }
        Variables vars;
        vars.ee_name = tip_frames_[0];
        vars.joint_group = group_name_;
        tf2::fromMsg(ik_poses[0], vars.target_pose);
        ObjectiveMaster om(robot_model_, vars);

        opt.set_min_objective(RelaxedIKPlugin::wrap, &om);
        opt.set_ftol_abs(0.0005);  // stop when function value is not improved by at least this
        opt.set_xtol_abs(0.0001);  // stop when all parameter changes add up to less than this

        solution = ik_seed_state;

        bool found_solution = false;
        std::chrono::duration<double> time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
        double time_remaining = timeout - time_elapsed.count();
        while (!found_solution && time_remaining > 0) {
            double val;
            try {
                opt.set_maxtime(time_remaining);
                opt.optimize(solution, val);
            } catch (const nlopt::roundoff_limited &) {
                RCLCPP_WARN(node_->get_logger(), "nlopt::roundoff_limited error, result might still be usable");
            } catch (std::exception &e) {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "An exception occured in RelaxedIK: " << e.what());
                return false;
            }

            if (options.return_approximate_solution) {
                found_solution = true;
            } else {
                // check linear and angular errors
                state_->setJointGroupPositions(vars.joint_group, solution);
                state_->updateLinkTransforms();
                const Eigen::Isometry3d current = state_->getGlobalLinkTransform(vars.ee_name);
                const double position_threshold = 0.001;
                const double orientation_threshold = 0.001;
                const double linear_distance = (current.translation() - vars.target_pose.translation()).norm();
                const Eigen::Quaterniond current_orientation(current.rotation());
                const Eigen::Quaterniond target_orientation(vars.target_pose.rotation());
                const double angular_distance = current_orientation.angularDistance(target_orientation);
                found_solution = (linear_distance < position_threshold && angular_distance < orientation_threshold);

                if (solution_callback && found_solution) {
                    solution_callback(tf2::toMsg(current), solution, error_code);
                    found_solution = (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
                }
            }

            if (!found_solution) {
                // randomize seed
                robot_model_->getVariableRandomPositions(state_->getRandomNumberGenerator(), solution);
            }

            time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
            time_remaining = timeout - time_elapsed.count();
        }

        if (found_solution) {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        } else {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        }

        return found_solution;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(relaxed_ik::RelaxedIKPlugin, kinematics::KinematicsBase);
