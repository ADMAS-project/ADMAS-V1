from setuptools import find_packages, setup

package_name = 'face_monitoring_publisher'

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
    maintainer='mohammed',
    maintainer_email='mohamedmed01345@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = face_monitoring_publisher.publisher:main',
            'subscriper_node = face_monitoring_publisher.publisher:listener',
        ],
    },
)
