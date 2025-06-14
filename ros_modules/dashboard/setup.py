from setuptools import find_packages, setup

package_name = 'dashboard'

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
    description='ROS 2 Dashboard package for driver UI and traffic sign processing',
    license='2025',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = dashboard.dashboard:main',
            'traffic_sign = dashboard.traffic_sign:main',
        ],
    },
)
