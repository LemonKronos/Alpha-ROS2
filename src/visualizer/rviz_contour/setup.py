from setuptools import find_packages, setup

package_name = 'rviz_contour'

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
    description='convert contour to marker visualizer rviz2',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rviz_contour = rviz_contour.rviz_contour:main',
            'rviz_contour_points = rviz_contour.rviz_contour_points:main'
        ],
    },
)
