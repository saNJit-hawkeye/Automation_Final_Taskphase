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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),  
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'srvpkg',  # This line tells Python to expect your service package
    ],
    zip_safe=True,
    maintainer='Sanjit',
    maintainer_email='sanjit@example.com',
    description='Task1 Robot Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_service = task1.waypoint_service:main',
            'move = task1.move:main',
            'client = task1.client:main',
        ],
    },
)
