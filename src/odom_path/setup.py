from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'odom_path'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aayush',
    maintainer_email='aayush@example.com',
    description='Simple node to convert odometry messages to path visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_path_node = odom_path.odom_path_node:main',
        ],
    },
)
