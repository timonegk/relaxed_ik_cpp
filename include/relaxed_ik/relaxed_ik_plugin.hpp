#pragma once
#include <moveit/kinematics_base/kinematics_base.h>
#include <relaxed_ik/objective.hpp>
#include <relaxed_ik_parameters.hpp>

namespace relaxed_ik {
    class RelaxedIKPlugin : public kinematics::KinematicsBase {
    public:
        moveit::core::RobotStatePtr state_;
        std::shared_ptr<ParamListener> parameter_listener_;

        bool initialize(const rclcpp::Node::SharedPtr &node,
                        const moveit::core::RobotModel &robot_model,
                        const std::string &group_name,
                        const std::string &base_frame,
                        const std::vector<std::string> &tip_frames,
                        double search_discretization) override;

        static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data);

        virtual bool searchPositionIK(
                std::vector<geometry_msgs::msg::Pose> const &ik_poses,
                std::vector<double> const &ik_seed_state,
                double timeout,
                std::vector<double> const &,
                std::vector<double> &solution,
                IKCallbackFn const &solution_callback,
                IKCostFn const &cost_function,
                moveit_msgs::msg::MoveItErrorCodes &error_code,
                kinematics::KinematicsQueryOptions const &options = kinematics::KinematicsQueryOptions(),
                moveit::core::RobotState const *context_state = nullptr) const;


        virtual bool getPositionFK(std::vector<std::string> const &,
                                   std::vector<double> const &,
                                   std::vector<geometry_msgs::msg::Pose> &) const {
            return false;
        }

        virtual bool getPositionIK(geometry_msgs::msg::Pose const &,
                                   std::vector<double> const &,
                                   std::vector<double> &,
                                   moveit_msgs::msg::MoveItErrorCodes &,
                                   kinematics::KinematicsQueryOptions const &) const {
            return false;
        }

        virtual bool searchPositionIK(geometry_msgs::msg::Pose const &ik_pose,
                                      std::vector<double> const &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      moveit_msgs::msg::MoveItErrorCodes &error_code,
                                      kinematics::KinematicsQueryOptions const &options =
                                      kinematics::KinematicsQueryOptions()) const {
            return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                    ik_seed_state,
                                    timeout,
                                    std::vector<double>(),
                                    solution,
                                    IKCallbackFn(),
                                    error_code,
                                    options);
        }

        virtual bool searchPositionIK(geometry_msgs::msg::Pose const &ik_pose,
                                      std::vector<double> const &ik_seed_state,
                                      double timeout,
                                      std::vector<double> const &consistency_limits,
                                      std::vector<double> &solution,
                                      moveit_msgs::msg::MoveItErrorCodes &error_code,
                                      kinematics::KinematicsQueryOptions const &options =
                                      kinematics::KinematicsQueryOptions()) const {
            return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                    ik_seed_state,
                                    timeout,
                                    consistency_limits,
                                    solution,
                                    IKCallbackFn(),
                                    error_code,
                                    options);
        }

        virtual bool searchPositionIK(geometry_msgs::msg::Pose const &ik_pose,
                                      std::vector<double> const &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      IKCallbackFn const &solution_callback,
                                      moveit_msgs::msg::MoveItErrorCodes &error_code,
                                      kinematics::KinematicsQueryOptions const &options =
                                      kinematics::KinematicsQueryOptions()) const {
            return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                    ik_seed_state,
                                    timeout,
                                    std::vector<double>(),
                                    solution,
                                    solution_callback,
                                    error_code,
                                    options);
        }

        virtual bool searchPositionIK(geometry_msgs::msg::Pose const &ik_pose,
                                      std::vector<double> const &ik_seed_state,
                                      double timeout,
                                      std::vector<double> const &consistency_limits,
                                      std::vector<double> &solution,
                                      IKCallbackFn const &solution_callback,
                                      moveit_msgs::msg::MoveItErrorCodes &error_code,
                                      kinematics::KinematicsQueryOptions const &options =
                                      kinematics::KinematicsQueryOptions()) const {
            return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                    ik_seed_state,
                                    timeout,
                                    consistency_limits,
                                    solution,
                                    solution_callback,
                                    error_code,
                                    options);
        }

        virtual bool searchPositionIK(
                std::vector<geometry_msgs::msg::Pose> const &ik_poses,
                std::vector<double> const &ik_seed_state,
                double timeout,
                std::vector<double> const &consistency_limits,
                std::vector<double> &solution,
                IKCallbackFn const &solution_callback,
                moveit_msgs::msg::MoveItErrorCodes &error_code,
                kinematics::KinematicsQueryOptions const &options = kinematics::KinematicsQueryOptions(),
                moveit::core::RobotState const *context_state = NULL) const {
            return searchPositionIK(ik_poses,
                                    ik_seed_state,
                                    timeout,
                                    consistency_limits,
                                    solution,
                                    solution_callback,
                                    IKCostFn(),
                                    error_code,
                                    options,
                                    context_state);
        }

        virtual std::vector<std::string> const &getJointNames() const { return joint_names_; }

        virtual std::vector<std::string> const &getLinkNames() const { return link_names_; }

        Variables vars_;
        std::unique_ptr<ObjectiveMaster> om_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> link_names_;
        const moveit::core::JointModelGroup *jmg_;
    };


}