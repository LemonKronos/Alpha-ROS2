from setuptools import find_packages, setup

package_name = 'launch_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch_utils/lidar_2d_handler_launch.py',
            'launch_utils/virtual_control_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trong Data',
    maintainer_email='trongdat4work@gmail.com',
    description='UHM',
    license='Proprietary',
    entry_points={
        'console_scripts': [],
    },
)
