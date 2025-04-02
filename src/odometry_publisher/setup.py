from setuptools import find_packages, setup

package_name = 'odometry_publisher'

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
    maintainer='roombazon',
    maintainer_email='mehndirattakeshav@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'odometry_publisher = odometry_publisher.odometry_publisher:main',
		'serial_odom = odometry_publisher.serial_odom_node:main',
		'joystick_node = odometry_publisher.joystick_node:main',
        ],
    },
)
