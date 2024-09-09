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

    class IKCostFnGoal : public Objective {
	    public:
		    IKCostFnGoal(const geometry_msgs::msg::Pose &pose, const kinematics::KinematicsBase::IKCostFn &fn) : fn_(fn), pose_(pose) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
	    private:
	const kinematics::KinematicsBase::IKCostFn fn_;
	const geometry_msgs::msg::Pose pose_;
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
