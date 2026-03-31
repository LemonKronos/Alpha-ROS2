from setuptools import find_packages, setup

package_name = 'vfh_sphere'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mr_lemon',
    maintainer_email='trongdat4work@gmail.com',
    description='Visualizer for VFH table',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vfh_sphere = vfh_sphere.vfh_sphere:main'
        ],
    },
)
