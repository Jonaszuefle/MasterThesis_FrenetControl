// internal calculations in omni coordinate system, publishing and subscribing: conversion to RViz coordinate system
// only messages are in RViz coordinate system, all calculations and variables are in omni coordinate system
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <urdf/model.h>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <unistd.h>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>
//#include <geometry_msgs/TransformStamped.h>

#include "omni_msgs/msg/hid_feedback.hpp"
#include "omni_msgs/msg/hid_state.hpp"
#include <pthread.h>

#include <vector>
#include <iostream>

float prev_time;
int calibrationStyle;
double omni2rviz=0.03;
double angle_scalpel=39.0; //degree
double length_blade=0.23; //RViz length units


// define struct which contains all measurable parameters
struct OmniState
{
    hduVector3Dd position;                          // mm -- Cartesian
    hduVector3Dd velocity;                          // mm/s -- Cartesian

    hduMatrix transform;
    std::vector<double> angular_velocity;           // rad/s -- velocity of gimbal
    std::vector<double> joint_angles;               // rad
    std::vector<double> gimbal_angles;              // rad
    
    hduVector3Dd force;                             // N -- Cartesian
    hduVector3Dd torque;                            // mNm -- Cartesian
    std::vector<double> joint_torque;               // mNm -- Joint Space
    std::vector<double> gimbal_torque;              // mNm -- Joint Space
 
    bool feedback_mode;                             // feedback type: force: false, positional true
    hduVector3Dd desired_position;                  // desired position feedback
    hduVector3Dd desired_force;                     
    hduVector3Dd force_output;

    bool buttons[2];                                // state of the two buttons
    bool prev_buttons[2];                           // previous state
    bool buttons_mode[2];                           // mode (on/off)
    double units_ratio;

    

    // MyError
    hduVector3Dd sum_errors;
    float time_step;
    
};



class PhantomROS
{

public:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<omni_msgs::msg::HIDState>::SharedPtr hid_state_publisher;

    rclcpp::Subscription<omni_msgs::msg::HIDFeedback>::SharedPtr feedback_sub;

    rclcpp::Clock::SharedPtr clock_;                // Create Clock objekt for time stamp

    std::string device_name, ref_frame, units;

    int publish_rate;

    OmniState *state;       // pointer on struct OmniState
    
    rclcpp::TimerBase::SharedPtr pub_timer;

    PhantomROS(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        clock_ = node_->get_clock();
        node_->declare_parameter<std::string>("deviceName", "master");
        node_->declare_parameter<std::string>("referenceFrame", "HID0");
        node_->declare_parameter<std::string>("Units", "mm");
        node_->declare_parameter<int>("publishRate", 50);
        node_->get_parameter<std::string>("deviceName", device_name);
        node_->get_parameter<std::string>("referenceFrame", ref_frame);
        node_->get_parameter<std::string>("Units", units);
        node_->get_parameter<int>("publishRate", publish_rate);

    }

   
    // define publisher/subscriber and initalize the OmniState object 
    void init(OmniState *s)
    {
        // Subscribe to NAME/omni_feedback
        std::ostringstream stream1;
        stream1 << device_name << "/feedback";
        std::string feedback_topic = std::string(stream1.str());
        feedback_sub = node_->create_subscription<omni_msgs::msg::HIDFeedback>(feedback_topic.c_str(), 1, std::bind(&PhantomROS::feedback_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), ("listening to: " + std::string(feedback_topic) + " for haptic feedback").c_str());

        // Publish on NAME/omni_info
        std::ostringstream stream2;
        stream2 << device_name << "/state";
        std::string omni_info_topic_name = std::string(stream2.str());
        hid_state_publisher = node_->create_publisher<omni_msgs::msg::HIDState>(omni_info_topic_name.c_str(), 1);
        RCLCPP_INFO(node_->get_logger(), ("Publishing HID State on: " + std::string(omni_info_topic_name)).c_str());


        state = s;      // pointer on OmniState object

        state->buttons[0] = false;
        state->buttons[1] = false;
        state->prev_buttons[0] = false;
        state->prev_buttons[1] = false;
        state->buttons_mode[0] = false;
        state->buttons_mode[1] = false;
        state->feedback_mode = false;

        // Feedback init
        state->desired_position[0] = 0;
        state->desired_position[1] = 0;
        state->desired_position[2] = 0;

        state -> desired_force[0] = 0;
        state -> desired_force[1] = 0;
        state -> desired_force[2] = 0;

        // MyError
        state->sum_errors[0] = 0.0;
        state->sum_errors[1] = 0.0;
        state->sum_errors[2] = 0.0;
        state->time_step = 1.0/publish_rate;



        if (!units.compare("mm"))
            state->units_ratio = 1.0;
        else if (!units.compare("cm"))
            state->units_ratio = 10.0;
        else if (!units.compare("dm"))
            state->units_ratio = 100.0;
        else if (!units.compare("m"))
            state->units_ratio = 1000.0;
        else
        {
            state->units_ratio = 1.0;
            RCLCPP_WARN(node_->get_logger(), "Unknown units [%s] unsing [mm]", units.c_str());
            units = "mm";
        }
        RCLCPP_INFO(node_->get_logger(), "HID position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
        RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "Publishing Omni state at [%d] Hz", publish_rate);
        pub_timer = node_->create_wall_timer(std::chrono::milliseconds(int(1000/publish_rate)), std::bind(&PhantomROS::publish_hid_state, this));
 
    }


    /*******************************************************************************
     ROS node callback.
     *******************************************************************************/

    void feedback_callback(const omni_msgs::msg::HIDFeedback::SharedPtr omnifeed)
    {
        // feedback type
        if (omnifeed->feedback_mode==1) {
            state->feedback_mode = false; // feedback type: -1 position, 0 unchanged, 1 force
        }
        else if (omnifeed->feedback_mode==-1) {
            state->feedback_mode = true;
        }
        // position feedback
        state->desired_position[0] = omnifeed->desired_position.x;
        state->desired_position[1] = omnifeed->desired_position.z;
        state->desired_position[2] = omnifeed->desired_position.y*-1.0;

        // force feedback
        state->desired_force[0] = omnifeed->desired_force.x;
        state->desired_force[1] = omnifeed->desired_force.z; 
        state->desired_force[2] = omnifeed->desired_force.y*-1.0;

        // torque feedback may be added

    }

    void publish_hid_state()
    {
        // Build the info msg
        omni_msgs::msg::HIDState info_msg;

        // create time stamp in header
        info_msg.header.stamp = clock_->now();
        info_msg.header.frame_id = ref_frame;

        // Buttons pressed?
        info_msg.grey_button_pressed = state->buttons[0];
        info_msg.white_button_pressed = state->buttons[1];

        // Button mode: is 0 or 1, changes through press
        for (int i = 0; i < 2; i++) {
            if (state->buttons[i] != state->prev_buttons[i]){
                if (state->buttons[i] == true){
                    state->buttons_mode[i] = !state->buttons_mode[i];
                    if (i==1) {
                        state->feedback_mode = !state->feedback_mode;
                        if (state->feedback_mode == 1) {
                            RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "position feedback mode");
                        }
                        else {
                            RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "force feedback mode");
                        }
                    }
                }
                state->prev_buttons[i] = state->buttons[i];
            }
        }

        info_msg.grey_button_mode = state->buttons_mode[0];
        info_msg.white_button_mode = state->buttons_mode[1];
        
        // Position (change y and z axis in order to make it more intuitive - see reading coordinates in omni_state_callback)
        // rotate around x-axis by -90 degrees
        hduMatrix transform_rotate_x={1,0,0,0,
                                      0,0,1,0,
                                      0,-1,0,0,
                                      0,0,0,1};
        hduMatrix rotated_transform = state->transform*transform_rotate_x;

        // Position
        info_msg.pose.position.x = rotated_transform[3][0] / 1000;
        info_msg.pose.position.y = rotated_transform[3][1] / 1000;
        info_msg.pose.position.z = rotated_transform[3][2] / 1000;

        // Orientation
        hduQuaternion get_quaternion(rotated_transform);
        info_msg.pose.orientation.x = get_quaternion.v()[0];
        info_msg.pose.orientation.y = get_quaternion.v()[1];
        info_msg.pose.orientation.z = get_quaternion.v()[2];
        info_msg.pose.orientation.w = get_quaternion.s();

        // Velocity
        info_msg.velocity.x = state->velocity[0] / 1000;
        info_msg.velocity.y = state->velocity[2]*-1.0 / 1000;
        info_msg.velocity.z = state->velocity[1]/ 1000;

        // Tip Pose
        hduMatrix translation_shaft = hduMatrix::createTranslation(0.0, 0.0, -40);
        hduMatrix rotation_blade = hduMatrix::createRotationAroundY(39.0 / 180.0 * M_PI);
        hduMatrix translation_blade = hduMatrix::createTranslation(0.0, 0.0, -0.23/omni2rviz);      // measured in RViz
        hduMatrix rotation_blade_z = hduMatrix::createRotationAroundZ(-45 * M_PI / 2);              // remove offset of scalpel orientation
        hduMatrix tool_tip_transform =  translation_blade  * rotation_blade * rotation_blade_z * rotated_transform; // * translation_shaft
        
        // Position
        info_msg.tool_tip_pose.position.x = tool_tip_transform[3][0];
        info_msg.tool_tip_pose.position.y = tool_tip_transform[3][1];
        info_msg.tool_tip_pose.position.z = tool_tip_transform[3][2];

        // Orientation
        hduQuaternion tip_quaternion(tool_tip_transform);
        info_msg.tool_tip_pose.orientation.x = tip_quaternion.v()[0];
        info_msg.tool_tip_pose.orientation.y = tip_quaternion.v()[1];
        info_msg.tool_tip_pose.orientation.z = tip_quaternion.v()[2];
        info_msg.tool_tip_pose.orientation.w = tip_quaternion.s();

        // Angular Velocity
        info_msg.angular_velocity = state->angular_velocity;

        // Joint Angles
        info_msg.joint_angles = state->joint_angles;

        // Gimbal Angles
        info_msg.gimbal_angles = state->gimbal_angles;

        // Force
        info_msg.force.x = state->force[0];
        info_msg.force.y = state->force[2]*-1.0;
        info_msg.force.z = state->force[1];

        // Torque
        info_msg.torque.x = state->torque[0];
        info_msg.torque.y = state->torque[1];
        info_msg.torque.z = state->torque[2];

        // Joint Torque
        info_msg.joint_torque = state -> joint_torque;

        // Gimbal Torque
        info_msg.gimbal_torque = state -> gimbal_torque;

        hid_state_publisher->publish(info_msg);

    }
};


// Callback to read data from touch device
HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
    OmniState *omni_state = static_cast<OmniState *>(pUserData);        // define pointer omni_state to access OmniState parameters
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)            // points on same position as state
    {
        RCLCPP_DEBUG(rclcpp::get_logger("omni_state_node"), "Updating calibration...");
        hdUpdateCalibration(calibrationStyle);
    }
    hdBeginFrame(hdGetCurrentDevice());
    

    /***********************************************************
     SET new FEEDBACK values (force or position)
     ***********************************************************/

    // position feedback
    if (omni_state->feedback_mode){

        float Kp = 0.05; // *1.5;
        float Ki = 0.0013275; //*10;

        hduVector3Dd error = omni_state->desired_position - omni_state->position;
        omni_state->sum_errors = omni_state->sum_errors + (error ) * omni_state->time_step;
        omni_state->force_output = Kp * (omni_state->desired_position - omni_state->position) *0 + 0* Ki * omni_state->sum_errors;
    }
    // force feedback
    else {
        omni_state->force_output = omni_state->desired_force;

    }
    
    omni_state->force_output[1] = omni_state->force_output[1] + (9.8*0.045);        // add weight of pencil

    hdSetDoublev(HD_CURRENT_FORCE, omni_state->force_output);       // set force on omni

    // Torque feedback may be added -> can be used instead of force

    /***********************************************************
     GET new STATE values
     ***********************************************************/

    // Get buttons
    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

    // Get Transform of endeffektor to pencil
    hduMatrix get_transform;
    hdGetDoublev(HD_CURRENT_TRANSFORM, get_transform);
    omni_state->transform = get_transform;

    // Set Position
    omni_state->position = {get_transform[3][0], get_transform[3][1], get_transform[3][2]};    


    // Get Velocity : This value is smoothed to reduce high frequency jitter.
    double get_velocity[3];   
    hdGetDoublev(HD_CURRENT_VELOCITY, get_velocity);
    omni_state->velocity = {get_velocity[0],get_velocity[1],get_velocity[2]};

    // Get Force
    double get_force[3];   
    hdGetDoublev(HD_CURRENT_FORCE, get_force);
    omni_state->force={get_force[0],get_force[1],get_force[2]};

    // Get Torque
    double get_torque[3];
    hdGetDoublev(HD_CURRENT_TORQUE, get_torque);
    omni_state->torque = {get_torque[0],get_torque[1],get_torque[2]};

    // Get Joint Torque
    double get_joint_torque[3];
    hdGetDoublev(HD_CURRENT_JOINT_TORQUE, get_joint_torque);
    omni_state->joint_torque = {get_joint_torque[0],get_joint_torque[1],get_joint_torque[2]};

    // Get Gimbal Torque
    double get_gimbal_torque[3];
    hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, get_gimbal_torque);
    omni_state->gimbal_torque = {get_gimbal_torque[0],get_gimbal_torque[1],get_gimbal_torque[2]};


    // Get Angular Velocity (Seems that it does not work well)
    double get_angular_velocity[3];   
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, get_angular_velocity);
    omni_state->angular_velocity = {get_angular_velocity[0],get_angular_velocity[1],get_angular_velocity[2]};

    // Get Joint Angles
    double get_joint_angles[3];   
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, get_joint_angles);
    omni_state->joint_angles = {get_joint_angles[0],get_joint_angles[1],get_joint_angles[2]};

    // Get Gimbal Angles
    double get_gimbal_angles[3];   
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, get_gimbal_angles);
    omni_state->gimbal_angles = {get_gimbal_angles[0],get_gimbal_angles[1],get_gimbal_angles[2]};


    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during main scheduler callback");
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration()
{
    int supportedCalibrationStyles;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "HD_CALIBRATION_ENCODER_RESET..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "HD_CALIBRATION_INKWELL..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle = HD_CALIBRATION_AUTO;
        RCLCPP_INFO(rclcpp::get_logger("omni_haptic_ndoe"), "HD_CALIBRATION_AUTO..");
    }
    if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
    {
        do
        {
            hdUpdateCalibration(calibrationStyle);
            RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "Calibrating.. (put stylus in well)");
            if (HD_DEVICE_ERROR(error = hdGetError()))
            {
                hduPrintError(stderr, &error, "Reset encoders reset failed.");
                break;
            }
        } while (hdCheckCalibration() != HD_CALIBRATION_OK);
        RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "Calibration complete.");
    }
    while (hdCheckCalibration() != HD_CALIBRATION_OK)
    {
        usleep(1e6);
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
            RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "Please place the device into the inkwell for calibration");
        else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
        {
            RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "Calibration updated successfully");
            hdUpdateCalibration(calibrationStyle);
        }
        else
            RCLCPP_FATAL(rclcpp::get_logger("omni_state_node"), "Unknown calibration status");
    }
}

int main(int argc, char **argv)
{
    ////////////////////////////////////////////////////////////////
    // Init Phantom
    ////////////////////////////////////////////////////////////////
    HDErrorInfo error;
    HHD hHD;
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("omni_state_node"), "Failed to initialize haptic device"); //: %s", &error);
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("omni_state_node"), "Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("omni_state_node"), "Failed to start the scheduler"); //, &error);
        return -1;
    }
    HHD_Auto_Calibration();

    ////////////////////////////////////////////////////////////////
    // Init ROS
    ////////////////////////////////////////////////////////////////
    
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("omni_state_node");
    OmniState state;

    PhantomROS omni_ros(node);

    omni_ros.init(&state);
    hdScheduleAsynchronous(omni_state_callback, &state,
                           HD_MAX_SCHEDULER_PRIORITY);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(node->get_logger(), "Ending Session....");
    hdStopScheduler();
    hdDisableDevice(hHD);

    return 0;
}
