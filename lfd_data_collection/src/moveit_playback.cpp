// #include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

class MoveitPlayback : public rclcpp::Node
{
  public:
    MoveitPlayback()
    : Node("moveit_update")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/gripper/pose", 1, std::bind(&MoveitPlayback::topic_callback, this, _1));

      subscription_2 = this->create_subscription<std_msgs::msg::Bool>(
      "/gripper/apple_grasp", 10, std::bind(&MoveitPlayback::topic_callback2, this, _1));
    }


  private:

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_2;
    bool apple_grasp = false;

    void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->orientation.w).c_str());

      // Create collision object for the robot to avoid
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = "apple_proxy";
      // collision_object.header.frame_id = "world";
      collision_object.id = "LFD_gripper";

      Eigen::Vector3d b(0.001, 0.001, 0.001);
      shapes::Mesh* m = shapes::createMeshFromResource("package://gripper/meshes/visual/low_poly_LFD_assembly.stl",b); 

      shape_msgs::msg::Mesh gripper_mesh;
      shapes::ShapeMsg gripper_mesh_msg;
      shapes::constructMsgFromShape(m,gripper_mesh_msg);
      gripper_mesh = boost::get<shape_msgs::msg::Mesh>(gripper_mesh_msg);
      
      geometry_msgs::msg::Pose gripper_pose = *msg;

      collision_object.meshes.push_back(gripper_mesh);
      collision_object.mesh_poses.push_back(gripper_pose);
      collision_object.operation = collision_object.ADD;

      

      // add the apple to the scene
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the sphere in meters
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1);
      primitive.dimensions[primitive.SPHERE_RADIUS] = .0375;

      geometry_msgs::msg::Pose apple_pose = *msg;

      if (apple_grasp){ //if the apple is grasped, follow the gripper

        // Define the pose of the apple (gripper pose + offset)
        double offsetV[4] = {0, 0, .43, 0};
        double gripperQ[4] = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        double gripperConj[4] = {msg->orientation.w, -msg->orientation.x, -msg->orientation.y, -msg->orientation.z};
        double qv[4];
        multiplyQuaternions(gripperQ, offsetV, qv);
        double result[4];
        multiplyQuaternions(qv, gripperConj, result);

        apple_pose.position.x = result[1] + msg->position.x;
        apple_pose.position.y = result[2] + msg->position.y;
        apple_pose.position.z = result[3] + msg->position.z;
      }else{
        
        apple_pose.position.x = 0.46;
        apple_pose.position.y = 0.46;
        apple_pose.position.z = .17;

        apple_pose.orientation.x = 0;
        apple_pose.orientation.y = 0;
        apple_pose.orientation.z = 0;
        apple_pose.orientation.w = 1;
      }
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(apple_pose);
        collision_object.operation = collision_object.ADD;

      // Add the collision object to the scene
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.applyCollisionObject(collision_object);

    }

    void topic_callback2(const std_msgs::msg::Bool::SharedPtr msg) {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->data).c_str());
      apple_grasp = msg->data;
    }

    void multiplyQuaternions(const double q1[4], const double q2[4], double result[4]) {
      result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
      result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
      result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
      result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitPlayback>());
  rclcpp::shutdown();
  return 0;
}
