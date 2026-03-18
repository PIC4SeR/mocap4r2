from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os
import yaml

package_name = "mocap4r2_robot_localization"

'''
Used to load parameters for composable nodes
'''
def dump_params(path, name):
    # Load the parameters specific to your ComposableNode
    with open(path, 'r') as file:
        return [yaml.safe_load(file)[name]['ros__parameters']]

def generate_launch_description():
  namespace = LaunchConfiguration('namespace')

  mocap4r2_robot_localization = Node(
    package=package_name,
    executable='localization_program',
    name='mocap4r2_robot_localization',
    output='screen',
    namespace=namespace,
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    parameters=[os.path.join(get_package_share_directory(package_name), 'params', 'params.yaml')]
  )
  declare_namespace_cmd = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Namespace for the nodes'
  )

  return LaunchDescription([declare_namespace_cmd, 
                            mocap4r2_robot_localization])
