from setuptools import find_packages, setup

package_name = 'launch_utils'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch_utils/ros_gz_bridge_launch.py',
            'launch_utils/static_tf_launch.py',
            'launch_utils/spatial_container_launch.py',
            'launch_utils/full_launch.py',
            'launch_utils/spatial_observation.py',
            'launch_utils/training_launch.py'
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
