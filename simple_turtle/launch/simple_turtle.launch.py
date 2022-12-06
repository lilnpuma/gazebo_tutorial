import os
import launch
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import time
def generate_launch_description():
    package_file_dir = get_package_share_directory('simple_turtle')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    
    # Create parameters for the nodes
    world = launch.substitutions.LaunchConfiguration('world')
    record_enabled = launch.substitutions.LaunchConfiguration('record_enabled')
    replay_only = launch.substitutions.LaunchConfiguration('replay_only')
    bag_file = launch.substitutions.LaunchConfiguration('bag_file')
    # Create the launch arguments
    bag_file_arg = launch.actions.DeclareLaunchArgument('bag_file', default_value='recorder'+str(time.time())+'.bag')
    replay_only_arg = launch.actions.DeclareLaunchArgument('replay_only', default_value='False')
    record_enabled_arg = launch.actions.DeclareLaunchArgument('record_enabled', default_value='False')
    world_args = launch.actions.DeclareLaunchArgument('world', default_value=os.path.join(package_file_dir, 'worlds/maze_world2.world'))

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    x_pose = launch.substitutions.LaunchConfiguration('x_pose', default='0.0')
    y_pose = launch.substitutions.LaunchConfiguration('y_pose', default='0.0')

    # Start Gazebo server
    gzserver_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    gzclient_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    # Start Robot state publisher
    robot_state_publisher_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    # Spawn TurtleBot3
    spawn_turtlebot_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    # Start TurtbleBot3 controller
    controller = launch_ros.actions.Node(
        condition = launch.conditions.IfCondition(launch.substitutions.PythonExpression(['not ', replay_only])),
        package = 'simple_turtle',
        executable = 'controller',
        name='turtle_controller',
        output = 'screen',
        parameters = [{'world': world}]
        )
    # Create an executable that will run the bag recorder for 
    recorder = launch.actions.ExecuteProcess(
        # Conditionally run the recorder based on the launch argument
        condition = launch.conditions.IfCondition(launch.substitutions.PythonExpression([record_enabled, ' and ', 'not ', replay_only])),
        cmd = ['ros2', 'bag', 'record', '-a' , '-x','/camera/*', '-o', bag_file],
        output = 'screen')


    bag_player = launch.actions.ExecuteProcess(
        # Conditionally run the bag player based on the launch argument
        condition = launch.conditions.IfCondition(launch.substitutions.PythonExpression([replay_only])),
        cmd = ['ros2', 'bag', 'play', bag_file, '--topics', '/cmd_vel'],
        output = 'screen')
    
    return launch.LaunchDescription([
        # Add the launch arguments
        bag_file_arg,
        replay_only_arg,
        record_enabled_arg,
        world_args,
        # Everything to launch the robot
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        bag_player,
        # Add the nodes to the launch description
        controller,
        # Add the recorder to the launch description
        recorder,
        ])