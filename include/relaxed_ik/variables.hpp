#pragma once
#include <string>
#include <Eigen/Eigen>
#include <moveit/planning_scene/planning_scene.h>

namespace relaxed_ik {
    struct Variables {
        Eigen::Isometry3d target_pose;
        std::string ee_name;
        std::string joint_group;
        planning_scene::PlanningSceneConstPtr planning_scene;
	std::vector<double> seed_state;
    };
}
