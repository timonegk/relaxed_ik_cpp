#pragma once

#include <vector>
#include <cstddef>
#include <moveit/robot_state/robot_state.h>
#include <relaxed_ik/variables.hpp>

namespace relaxed_ik {

    class Objective {
    public:
        virtual double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) = 0;
    };

    class MatchEEPosiDoF : public Objective {
    public:
        MatchEEPosiDoF(int axis) : axis(axis) {}
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        int axis;
    };

    class MatchEERotaDoF : public Objective {
    public:
        MatchEERotaDoF(int axis) : axis(axis) {}
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

    class ObjectiveMaster {
    public:
        ObjectiveMaster(const moveit::core::RobotModelConstPtr &m, Variables vars);
        double call(const std::vector<double> &joints, std::vector<double> &grad);
    private:
        std::vector<std::unique_ptr<Objective>> objectives_;
        std::vector<double> weights_;
        moveit::core::RobotStatePtr state_;
        const Variables vars_;
    };
}