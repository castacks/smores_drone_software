import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'depth_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('depth_perception/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='iyer.abhishek18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'thermal_pub_sub = depth_perception.publish_thermals:main', # executable_name = package_name.file_name:entry_function
            'stereobm = depth_perception.sync_stereo_pair:main',
            'moge_infer_depth = depth_perception.moge_inference_mono:main',
            'madpose = depth_perception.madpose_metric_solver:main',
            # 'pclsub = depth_perception.pcl_sub:main'
        ],
    },
)

# Run node with
# ros2 run package_name executable_name
