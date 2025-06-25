import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='smb_autonomous',
            executable='state_machine',
            name='autonomous_exploration_state_machine'),
  ])