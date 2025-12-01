from setuptools import setup

package_name = 'rosbag_image_extractor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SMORES Team',
    maintainer_email='smores@example.com',
    description='Extract images from ROS2 bag files to MP4 or individual frames',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extract_images = rosbag_image_extractor.extract_images:main',
        ],
    },
)
