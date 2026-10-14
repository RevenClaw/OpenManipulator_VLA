from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'custom_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Custom user package for OpenManipulator-X',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_home_position = custom_package.record_home_position:main',
            'interactive_homing_node = custom_package.interactive_homing_node:main',
            'log_joint_states = custom_package.log_joint_states:main',
            'joint_commander = custom_package.joint_commander:main',
            'ee_commander = custom_package.ee_commander:main',
        ],
    },
)
