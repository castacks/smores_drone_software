from setuptools import find_packages, setup

package_name = 'static_transform_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smores',
    maintainer_email='aayushf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "static_transform_pub_node = static_transform_publisher.static_transform_publisher.static_tf_pub_node.py:main"
        ],
    },
)
