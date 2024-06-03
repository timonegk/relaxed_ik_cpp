#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <random>
#include <fmt/format.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

const std::string CYLINDER_PATH = ament_index_cpp::get_package_share_directory("relaxed_ik_cpp") + "/test/cylinder.stl";
const std::string STICK_PATH = ament_index_cpp::get_package_share_directory("relaxed_ik_cpp") + "/test/stick.stl";

std::shared_ptr<hpp::fcl::CollisionObject> loadMesh(const std::string& file_name) {
    hpp::fcl::NODE_TYPE bv_type = hpp::fcl::BV_AABB;
    hpp::fcl::MeshLoader loader(bv_type);
    hpp::fcl::BVHModelPtr_t bvh = loader.load(file_name);
    return std::make_shared<hpp::fcl::CollisionObject>(bvh);
}

Eigen::Isometry3d get_rand_tf() {
    static std::mt19937 gen(0);
    std::uniform_real_distribution<> dist(-1, 1);
    Eigen::Isometry3d tf;
    tf.translation() = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
    Eigen::Quaterniond rot(dist(gen), dist(gen), dist(gen), dist(gen));
    rot.normalize();
    tf.linear() = rot.toRotationMatrix();
    return tf;
}


void test_hpp_fcl() {
    auto cylinder_obj = loadMesh(CYLINDER_PATH);
    auto stick_obj = loadMesh(STICK_PATH);
    {
        hpp::fcl::CollisionRequest req; hpp::fcl::CollisionResult res;
        req.enable_distance_lower_bound = true;
        auto n_contacts = hpp::fcl::collide(cylinder_obj.get(), stick_obj.get(), req, res);
        std::cout << "Distance: " << res.distance_lower_bound << std::endl;
    }

    std::chrono::duration<double> collision_time(0);
    std::chrono::duration<double> distance_time(0);
    int count = 100;
    for (int i = 0; i < count; ++i) {
        Eigen::Isometry3d tf = get_rand_tf();
        cylinder_obj->setTransform(tf.linear(), tf.translation());
        cylinder_obj->computeAABB();
        tf = get_rand_tf();
        stick_obj->setTransform(tf.linear(), tf.translation());
        stick_obj->computeAABB();
        {
            hpp::fcl::CollisionRequest req; hpp::fcl::CollisionResult res;
            auto start = std::chrono::high_resolution_clock::now();
            hpp::fcl::collide(cylinder_obj.get(), stick_obj.get(), req, res);
            collision_time += (std::chrono::high_resolution_clock::now() - start);
        }
        {
            hpp::fcl::CollisionRequest req; hpp::fcl::CollisionResult res;
            req.enable_distance_lower_bound = true;
            auto start = std::chrono::high_resolution_clock::now();
            hpp::fcl::collide(cylinder_obj.get(), stick_obj.get(), req, res);
            distance_time += (std::chrono::high_resolution_clock::now() - start);
        }
    }
    std::cout << fmt::format("HPP-FCL: Time for collision checking {}s, {}s per call", collision_time.count(), collision_time.count()/count) << std::endl;
    std::cout << fmt::format("HPP-FCL: Time for distance checking {}s, {}s per call", distance_time.count(), distance_time.count()/count) << std::endl;
}

void test_fcl() {
    auto cylinder_mesh = std::shared_ptr<const shapes::Mesh>(shapes::createMeshFromResource(std::string("file://") + CYLINDER_PATH));
    auto cylinder_model = moveit::core::LinkModel("cylinder", 1.0);
    auto cylinder_fcl_geom = collision_detection::createCollisionGeometry(cylinder_mesh, &cylinder_model, 0);
    auto cylinder_fcl_col = std::make_shared<fcl::CollisionObjectd>(cylinder_fcl_geom->collision_geometry_);

    auto stick_mesh = std::shared_ptr<const shapes::Mesh>(shapes::createMeshFromResource(std::string("file://") + STICK_PATH));
    auto stick_model = moveit::core::LinkModel("stick", 1.0);
    auto stick_fcl_geom = collision_detection::createCollisionGeometry(stick_mesh, &stick_model, 0);
    auto stick_fcl_col = std::make_shared<fcl::CollisionObjectd>(stick_fcl_geom->collision_geometry_);

    {
        fcl::CollisionRequestd req;
        fcl::CollisionResultd res;
        auto n_contacts = fcl::collide(cylinder_fcl_col.get(), stick_fcl_col.get(), req, res);
        fcl::DistanceRequestd dreq(false);
        fcl::DistanceResultd dres;
        double distance = fcl::distance(cylinder_fcl_col.get(), stick_fcl_col.get(), dreq, dres);
        std::cout << "Distance: " << distance << std::endl;
    }

    std::chrono::duration<double> collision_time(0);
    std::chrono::duration<double> distance_time(0);
    std::chrono::duration<double> distance_bin_time(0);
    const int count = 100;
    int distance_count = 0;
    for (int i = 0; i < count; ++i) {
        fcl::CollisionRequestd req;
	req.enable_contact = true;
        fcl::CollisionResultd res;
        fcl::DistanceRequestd dreq(false);
        fcl::DistanceResultd dres;
        Eigen::Isometry3d tf;
        cylinder_fcl_col->setTransform(get_rand_tf());
        cylinder_fcl_col->computeAABB();
        stick_fcl_col->setTransform(get_rand_tf());
        stick_fcl_col->computeAABB();
        auto start = std::chrono::high_resolution_clock::now();
        fcl::collide(cylinder_fcl_col.get(), stick_fcl_col.get(), req, res);
        collision_time += (std::chrono::high_resolution_clock::now() - start);
	if (!res.isCollision()) {
		distance_count++;
		start = std::chrono::high_resolution_clock::now();
		double real_distance = fcl::distance(cylinder_fcl_col.get(), stick_fcl_col.get(), dreq, dres);
		distance_time += (std::chrono::high_resolution_clock::now() - start);
	} else {
		std::cout << "Penetration depth: " << res.getContact(0).penetration_depth << std::endl;
	}

        /*start = std::chrono::high_resolution_clock::now();

        double diff = 0.4;
        double current_padding = 0.0;
        do {
            auto cylinder_check_geom = collision_detection::createCollisionGeometry(cylinder_mesh, 1.0,
                                                                                    current_padding,
                                                                                    &cylinder_model, 0);
            auto cylinder_check_col = std::make_shared<fcl::CollisionObjectd>(
                    cylinder_check_geom->collision_geometry_);
            cylinder_check_col->setTransform(cylinder_fcl_col->getTransform());
            cylinder_check_col->computeAABB();
            fcl::CollisionRequestd check_req;
            fcl::CollisionResultd check_res;
            bool collision = fcl::collide(cylinder_check_col.get(), stick_fcl_col.get(), check_req, check_res);
            if (collision) {
                // padding is too large
                if (current_padding < diff) break;  // cannot subtract
                current_padding -= diff;
            } else {
                current_padding += diff;
            }
            diff /= 2.0;
        } while (current_padding < 1.0 && diff > 0.001);
        double approx_distance = current_padding;
        distance_bin_time += (std::chrono::high_resolution_clock::now() - start);
        std::cout << fmt::format("Real distance: {:.6f}, approx distance: {:.6f}, diff {:.6f} ({:.2f}%)",
                                 real_distance, approx_distance,
                                 std::abs(real_distance - approx_distance),
                                 std::abs(real_distance - approx_distance) / real_distance * 100) << std::endl;*/
        //std::cout << "Distance: " << real_distance << std::endl;
    }
    std::cout << fmt::format("FCL: Time for collision checking {}s, {}s per call", collision_time.count(), collision_time.count()/count) << std::endl;
    std::cout << fmt::format("FCL: Time for distance checking {}s, {}s per call", distance_time.count(), distance_time.count()/distance_count) << std::endl;
}

int main() {
    test_fcl();
    test_hpp_fcl();
}
