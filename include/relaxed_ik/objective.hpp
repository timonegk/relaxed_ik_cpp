#pragma once

#include <utility>
#include <vector>
#include <cstddef>
#include <moveit/robot_state/robot_state.h>
#include <relaxed_ik/variables.hpp>
#include <moveit/collision_detection_fcl/collision_common.h>

namespace relaxed_ik {
    class Objective {
    public:
        virtual double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) = 0;
    };

    class MatchEEPosiDoF : public Objective {
    public:
        explicit MatchEEPosiDoF(int axis) : axis(axis) {}
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        int axis;
    };

    class MatchEERotaDoF : public Objective {
    public:
        explicit MatchEERotaDoF(int axis) : axis(axis) {}
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        int axis;
    };

    class MatchEEPosGoals : public Objective {
    public:
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    };

    class MatchEEQuatGoals : public Objective {
    public:
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    };

    class SelfCollision : public Objective {
    public:
        explicit SelfCollision(const moveit::core::RobotModelConstPtr &robot_model);
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        std::vector<collision_detection::FCLGeometryConstPtr> geoms_;
        std::map<std::string, std::vector<collision_detection::FCLCollisionObjectPtr>> fcl_objs_;
        collision_detection::AllowedCollisionMatrixConstPtr acm_;
    };

    class EnvCollisionDistance : public Objective {
    public:
        explicit EnvCollisionDistance(const planning_scene::PlanningSceneConstPtr &planning_scene) : planning_scene_(planning_scene) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningSceneConstPtr planning_scene_;
    };

    class EnvCollisionDepth : public Objective {
    public:
        explicit EnvCollisionDepth(const planning_scene::PlanningSceneConstPtr &planning_scene) : planning_scene_(planning_scene) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningSceneConstPtr planning_scene_;
    };

    class RCMGoal : public Objective {
    public:
        explicit RCMGoal(Eigen::Vector3d point) : point_(std::move(point)) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        const Eigen::Vector3d point_;
    };

    class RCMGoal2 : public Objective {
    public:
        RCMGoal2(const moveit::core::RobotModelConstPtr &robot_model, const Eigen::Vector3d &point);
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningScene planning_scene;
    };

    class LineGoal : public Objective {
    public:
        LineGoal(Eigen::Vector3d point, Eigen::Vector3d axis, std::string link_name) :
                point_(std::move(point)), axis_(std::move(axis)), link_name_(std::move(link_name)) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        const Eigen::Vector3d point_;
        const Eigen::Vector3d axis_;
        const std::string link_name_;
    };

    class AlignmentGoal : public Objective {
    public:
        AlignmentGoal(Eigen::Vector3d axis, std::string link_name) :
                axis_(std::move(axis)), link_name_(std::move(link_name)) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        const Eigen::Vector3d axis_;
        const std::string link_name_;
    };

    class ObjectiveMaster {
    public:
        ObjectiveMaster(const moveit::core::RobotModelConstPtr &m, Variables vars, const std::vector<std::pair<std::shared_ptr<Objective>, double>> &objectives);
        double call(const std::vector<double> &joints, std::vector<double> &grad);
    private:
        std::vector<std::pair<std::shared_ptr<Objective>, double>> objectives_;
        moveit::core::RobotStatePtr state_;
        const Variables vars_;
    };
}
