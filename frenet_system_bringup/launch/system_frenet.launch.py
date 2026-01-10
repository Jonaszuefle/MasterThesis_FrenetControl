from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PythonExpression



def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('automation_controller', default_value='mpc'))
    ld.add_action(DeclareLaunchArgument('human_controller', default_value='false'))
      
    ld.add_action(DeclareLaunchArgument('recorder', default_value='false'))    
    ld.add_action(DeclareLaunchArgument('fake_hardware', default_value='false'))      

    is_human_mpc = PythonExpression(["'", LaunchConfiguration('human_controller'), "' == 'mpc'"])
    is_human_pi = PythonExpression(["'", LaunchConfiguration('human_controller'), "' == 'pi'"])

    is_automation_mpc = PythonExpression(["'", LaunchConfiguration('automation_controller'), "' == 'mpc'"])
    is_automation_pi = PythonExpression(["'", LaunchConfiguration('automation_controller'), "' == 'pi'"])
    
    config = os.path.join(
        get_package_share_directory('frenet_system_bringup'),
        'config',
        'params.yaml'
    )
    rviz_config = os.path.join(
    get_package_share_directory('frenet_system_bringup'),
    'config',
    'frenet_config.rviz'
    )

    rviz_config2 = os.path.join(
    get_package_share_directory('frenet_system_bringup'),
    'config',
    'frenet_config2.rviz'
    )

    Control_Automation_PI = Node(
        package="frenet_control", 
        executable="pi_control",
        name="automation_control_node",
        parameters=[config,
                    {'human_control': False},
                    {'solver_name': 'mpc_automation'},
                    {'fake_hardware': LaunchConfiguration('fake_hardware')}],
        condition=IfCondition(is_automation_pi)
    )

    Control_Automation_MPC = Node(
        package="frenet_control", 
        executable="mpc_control",
        name="automation_control_node",
        parameters=[config,
                    {'human_control': False},
                    {'solver_name': 'mpc_automation'},
                    {'fake_hardware': LaunchConfiguration('fake_hardware')}],
        condition=IfCondition(is_automation_mpc)
    )

    Control_Human_MPC = Node(
        package="frenet_control", 
        executable="mpc_control",
        name="human_control_node",
        parameters=[config,
                    {'human_control': True},
                    {'solver_name': 'mpc_human'}],
        condition=IfCondition(is_human_mpc)
    )

    Control_Human_PI = Node(
        package="frenet_control", 
        executable="pi_control",
        name="human_control_node",
        parameters=[config,
                     {'human_control': True}],
        condition=IfCondition(is_human_pi)
    )


    Transformation = Node(
        package="frenet_transformation",
        executable="frenet_transformation",
        name="Transformation_Node",
        parameters=[config]
    )

    KUKA_force_transform = Node(
        package="human_force_transformation", 
        executable="human_force_transformation",
        name="force_trans_from_tool0_to_base",
        condition=UnlessCondition(LaunchConfiguration('fake_hardware'))
    )

    Position_Transformer = Node(
        package="real_hardware", 
        executable="real_hardware",
        name="position_trans_base_to_tool0",
        condition=UnlessCondition(LaunchConfiguration('fake_hardware'))
    )

    Visualization = Node(
        package="frenet_vis", 
        executable="frenet_vis",
        name="Visualization_Node",
        parameters=[config]
    )

    # launch rviz with safed settings 
    Rviz = Node(
        package= 'rviz2',
        executable= 'rviz2',
        name= 'rviz2',
        output= 'screen',
        arguments=['-d', rviz_config]
    )

    Rviz2 = Node(
        package= 'rviz2',
        executable= 'rviz2',
        name= 'rviz2',
        output= 'screen',
        arguments=['-d', rviz_config2]
    )


    Recorder = Node(
        package="ros_bag_recorder",
        executable="data_logger",
        name="ros_bag_logger",
        parameters=[config,
                    {'human_control': is_human_mpc or is_human_pi},
                    {'fake_hardware': LaunchConfiguration('fake_hardware')}],
        condition=IfCondition(LaunchConfiguration('recorder'))
    )

    
    System = TimerAction(
        period=5.0,
        actions=[
            Node(
            package="fake_hardware", 
            executable="fake_hardware",
            name="fake_system",
            parameters=[config,
                        {'human_control': is_human_mpc or is_human_pi}],
            condition=IfCondition(LaunchConfiguration('fake_hardware'))
            )]
    )
    

    ld.add_action(Control_Automation_MPC)
    ld.add_action(Control_Automation_PI)
    ld.add_action(Control_Human_PI)
    ld.add_action(Control_Human_MPC)
    ld.add_action(Transformation)
    ld.add_action(KUKA_force_transform)
    ld.add_action(Position_Transformer)
    ld.add_action(Visualization)
    ld.add_action(Rviz)
    ld.add_action(Rviz2)
    ld.add_action(System)
    ld.add_action(Recorder)
    
    return ld