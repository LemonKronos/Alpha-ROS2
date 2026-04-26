from setuptools import find_packages, setup

package_name = 'launch_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch_utils/alpha_brain_launch.py',
            'launch_utils/static_tf_launch.py',
            'launch_utils/lidar_2d_handler_launch.py',
            'launch_utils/pygame_control_launch.py',
            'launch_utils/advance_control_launch.py',
            'launch_utils/record_acrobatic_expert.py',
            'launch_utils/record_acrobatic_noob.py',
            'launch_utils/reactive_oa_launch.py',
            'launch_utils/depth_cam_test_launch.py',
            'launch_utils/ros_gz_bridge_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trong Data',
    maintainer_email='trongdat4work@gmail.com',
    description='launch files',
    license='Proprietary',
    entry_points={
        'console_scripts': [],
    },
)
