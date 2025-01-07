import os

from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法

from launch import LaunchDescription                 # launch文件的描述类
from launch.actions import IncludeLaunchDescription  # 节点启动的描述类
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction               # launch文件中的执行动作
from launch_ros.actions import PushRosNamespace      # ROS命名空间配置

from launch_ros.actions import Node     # 节点启动的描述类

def generate_launch_description():

    parameter_yaml = IncludeLaunchDescription(        # 包含指定路径下的另外一个launch文件
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('astra_camera'), 'launch'),
         '/astra_mini.launch.py'])
      )

    parameter_yaml_with_namespace = GroupAction(      # 对指定launch文件中启动的功能加上命名空间
        actions=[
        PushRosNamespace(''),
        parameter_yaml]
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
        ),

        Node(
            package='pubsub',
            executable='ImagePointCloudSub',
        ),

        Node(                             # 配置一个节点的启动
         package='rviz2',               # 节点所在的功能包
         executable='rviz2',            # 节点的可执行文件名
         name='rviz2',                  # 对节点重新命名
        ),

        parameter_yaml_with_namespace
        
    ])