from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'task1'  # Make sure this matches your package folder name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Automatically find packages
    data_files=[
        # Install marker file for package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob(package_name + '/urdf/*.urdf')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob(package_name + '/worlds/*.world')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(package_name + '/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # Replace with your actual name
    maintainer_email='your_email@example.com',  # Replace with your email
    description='bot Task1 Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = task1.move:main',  # This registers the move.py script
        ],
    },
)

