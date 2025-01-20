from setuptools import find_packages, setup

package_name = 'first_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amrit',
    maintainer_email='amrit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={  # Modern replacement for `tests_require`
        'test': ['pytest'],  # Add other testing dependencies if needed
    },
    entry_points={
        'console_scripts': [
            "test_node = first_controller.my_first_node:main",
            "circle = first_controller.draw_circle:main",
            "pose_subs = first_controller.pose_subscriber:main",
            "turtle_control = first_controller.turtle_control:main",
            "sum_int_pub = first_controller.sum_pub:main",
            "sum_int_subs = first_controller.sum_subs:main",
            "custom_msg_pub = first_controller.msgs_pub:main",
            "custom_msg_sub = first_controller.msgs_sub:main",
            "add_server = first_controller.add_two_ints_server:main",
            "add_client = first_controller.add_two_ints_client:main",
            "fib_action_server = first_controller.fibonacci_action_server:main",
            "fib_action_client = first_controller.fibonacci_action_client:main",
            "gazebo = first_controller.gazebo:main",
            "odom = first_controller.odom:main",
            "move = first_controller.move_point:main",
            "show_points = first_controller.show_point:main",
            "pco_apf = first_controller.pointcloud_apf:main",
            "new_pf = first_controller.new_apf:main",
            "fil_dat = first_controller.filtered_points:main",
            "depthreal = first_controller.depthrealsense:main",
            "dwa_ros = first_controller.dwa_ros:main",
            "depthros = first_controller.depthros:main"
        ],
    },
)
