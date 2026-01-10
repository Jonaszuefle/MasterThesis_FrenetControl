#include "real_hardware/real_hardware.hpp"
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include "tf2/exceptions.h"



JointCartesianTransform::JointCartesianTransform()
:Node("joint_cartesian_transform")
{
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&JointCartesianTransform::jointCallback, this, std::placeholders::_1)
    );

    omni_pub_ = this->create_publisher<omni_msgs::msg::HIDState>("master/state",10);

    
    // --- Pfad zum Package holen ---
    const auto package_share_dir = ament_index_cpp::get_package_share_directory("real_hardware");

    // --- Pfad zur URDF-Datei zusammensetzen ---
    const auto urdf_path = package_share_dir + "/resource/iiwa14.urdf";

    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open URDF file: %s", urdf_path.c_str());
    return;
    }

    std::stringstream urdf_stream;
    urdf_stream << urdf_file.rdbuf();
    std::string urdf_string = urdf_stream.str();
   

    if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
        return;
    }

    std::string base_link = "iiwa_link_0";
    std::string tip_link = "iiwa_link_ee";

    if (!kdl_tree_.getChain(base_link, tip_link, kdl_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from %s to %s", base_link.c_str(), tip_link.c_str());
        return;
    }

    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);

    joint_positions_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
    joint_velocities_ = KDL::JntArray(kdl_chain_.getNrOfJoints());

    first_state_transformed_ = false;
    initial_position_.pose.position.x = 0.0;
    initial_position_.pose.position.y = 0.0;
    initial_position_.pose.position.z = 0.0;
    initial_position_.velocity.x = 0.0;
    initial_position_.velocity.y = 0.0;
    initial_position_.velocity.z = 0.0;

}


void JointCartesianTransform::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{

    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        joint_positions_(i) = msg->position[i];
        joint_velocities_(i) = msg->velocity[i];
    }

    // FK
    KDL::Frame cart_pose;
    fk_solver_->JntToCart(joint_positions_, cart_pose);

    // Jacobian für Geschwindigkeit
    KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
    jac_solver_->JntToJac(joint_positions_, jacobian);
    KDL::Twist cart_vel;
    for(unsigned int i = 0; i < jacobian.columns(); ++i) {
    cart_vel += jacobian.getColumn(i) * joint_velocities_(i);
}

    // Float32MultiArray vorbereiten
    omni_msgs::msg::HIDState omni_msg;

    // Position
    omni_msg.pose.position.x = cart_pose.p.x();
    omni_msg.pose.position.y = cart_pose.p.y();
    omni_msg.pose.position.z = cart_pose.p.z();

    // Geschwindigkeit
    omni_msg.velocity.x = cart_vel.vel.x();
    omni_msg.velocity.y = cart_vel.vel.y();
    omni_msg.velocity.z = cart_vel.vel.z();

    
    
    if (!first_state_transformed_){
        initial_position_.pose.position = omni_msg.pose.position;
        first_state_transformed_ = true;
    }
    else{
        omni_msg.pose.position.x -= initial_position_.pose.position.x;
        omni_msg.pose.position.y -= initial_position_.pose.position.y;
        omni_msg.pose.position.z -= initial_position_.pose.position.z;

        omni_pub_->publish(omni_msg);
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointCartesianTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}