#pragma once
#include <string>
#include <Eigen/Eigen>

namespace relaxed_ik {
    struct Variables {
        Eigen::Isometry3d target_pose;
        std::string ee_name;
        std::string joint_group;
    };
}