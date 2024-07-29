#pragma once
#include <moveit/kinematics_base/kinematics_base.h>
#include <relaxed_ik/objective.hpp>

namespace relaxed_ik {
struct RelaxedIKKinematicsQueryOptions : public kinematics::KinematicsQueryOptions {
    std::vector<std::pair<std::shared_ptr<Objective>, double>> objectives_;
    RelaxedIKKinematicsQueryOptions();
    ~RelaxedIKKinematicsQueryOptions();
};
}
