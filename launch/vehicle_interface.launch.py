from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    interface_arg = DeclareLaunchArgument(
        'interface', 
        default_value='can0',
        description='CAN interface name'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='5.0',
        description='Max autonomous driving speed limit in km/h'
    )

    # 1. 接收器 (Lifecycle Node)
    socket_can_receiver = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver',
        namespace='',
        output='screen',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'use_bus_time': False,
            # 开启回环模式，允许监听本机发送的数据
            'enable_loopback': True
        }],
        remappings=[
            ('rx', '/from_can_bus'),
            ('can_rx', '/from_can_bus'),
            ('messages', '/from_can_bus')
        ]
    )

    # 2. 发送器 (Lifecycle Node)
    socket_can_sender = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender',
        namespace='',
        output='screen',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            # 尝试给发送器也开启回环 (部分版本支持)
            'enable_loopback': True
        }],
        remappings=[
            ('tx', '/to_can_bus'),
            ('can_tx', '/to_can_bus'),
            ('send', '/to_can_bus')
        ]
    )

    # 3. 驱动节点 (您的逻辑代码)
    teemo_driver = Node(
        package='teemo_vehicle_interface',
        executable='teemo_vehicle_interface_node',
        name='teemo_vehicle_interface',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed')
        }]
    )

    # 4. 自动配置和激活 Lifecycle 节点的辅助函数
    def configure_activate(node):
        return [
            # 启动后 -> Configure
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node,
                    on_start=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(node),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            ),
            # Inactive -> Activate
            RegisterEventHandler(
                event_handler=OnStateTransition(
                    target_lifecycle_node=node,
                    goal_state='inactive',
                    entities=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(node),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ],
                )
            )
        ]

    # 组装所有 Action
    ld = LaunchDescription()
    ld.add_action(interface_arg)
    ld.add_action(max_speed_arg)
    ld.add_action(socket_can_receiver)
    ld.add_action(socket_can_sender)
    ld.add_action(teemo_driver)

    # 添加自动激活事件
    for action in configure_activate(socket_can_receiver):
        ld.add_action(action)
        
    for action in configure_activate(socket_can_sender):
        ld.add_action(action)

    return ld
