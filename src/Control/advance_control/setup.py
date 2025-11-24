from setuptools import find_packages, setup

package_name = 'advance_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='mr_lemon',
    maintainer_email='trongdat4work@gmail.com',
    description='Drone control from phone for full 6 DoF',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'advance_control = advance_control.advance_control:main'
        ],
    },
)
