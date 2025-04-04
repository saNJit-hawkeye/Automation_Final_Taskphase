from setuptools import setup
import os
from glob import glob

package_name = 'robot_task1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file for package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join(package_name, 'urdf', '*.urdf'))),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join(package_name, 'worlds', '*.world'))),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join(package_name, 'launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanjit',
    maintainer_email='your_email@example.com',
    description='Robot Task1 Package for navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = robot_task1.move:main',
        ],
    },
)

