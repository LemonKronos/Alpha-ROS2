from setuptools import find_packages, setup

package_name = 'record_acrobatic'

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
    description='Record data set for AcrobaticOA',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'record_acrobatic = record_acrobatic.record_acrobatic:main',
            'add_nood_control = record_acrobatic.add_noob_control:main'
        ],
    },
)
