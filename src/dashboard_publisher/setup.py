from setuptools import find_packages, setup

package_name = 'dashboard_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'Pillow', 'cv_bridge'],
    zip_safe=True,
    maintainer='MoveIt Pro Maintainer',
    maintainer_email='support@picknik.ai',
    description='Example robot dashboard using PIL and sensor_msgs/msg/Image.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_publisher = dashboard_publisher.dashboard_publisher:main'
        ],
    },
)
