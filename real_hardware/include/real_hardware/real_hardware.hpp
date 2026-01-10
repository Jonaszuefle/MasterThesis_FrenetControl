#pragma once 

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/parameter_client.hpp>
#include "omni_msgs/msg/hid_state.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class JointCartesianTransform : public rclcpp::Node
{
public:
    JointCartesianTransform();

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<omni_msgs::msg::HIDState>::SharedPtr omni_pub_;

    bool first_state_transformed_;
    omni_msgs::msg::HIDState initial_position_;

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;


    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
};