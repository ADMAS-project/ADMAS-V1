from setuptools import find_packages, setup

package_name = 'emergency_parking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'numpy', 'std_msgs'],
    zip_safe=True,
    maintainer='x0AhmadAlaa',
    maintainer_email='ahmad.alaa.elkafrawy@gmail.com',
    description='ROS 2 Emergency Parking package',
    license='2025',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_parking_node = emergency_parking.emergency_parking_node:main',
        ],
    },
)
