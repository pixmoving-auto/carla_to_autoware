import os
from setuptools import setup
from glob import glob

package_name = 'carla_autoware_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dept',
    maintainer_email='zymouse@pixmoving.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "carla_to_autoware_detected_objects = carla_autoware_bridge.carla_to_autoware_detected_objects:main",
            "carla_to_autoware_localization = carla_autoware_bridge.carla_to_autoware_localization:main",
            "carla_to_autoware_vehicle_status = carla_autoware_bridge.carla_to_autoware_vehicle_status:main",
            "carla_to_autoware_waypoints = carla_autoware_bridge.carla_to_autoware_waypoints:main",
            "odometry_to_posestamped = carla_autoware_bridge.odometry_to_posestamped:main",
            "vehiclecmd_to_ackermanndrive = carla_autoware_bridge.vehiclecmd_to_ackermanndrive:main",
            "test_pub_node = carla_autoware_bridge.z_test_pub:main",
            "test_sub_node = carla_autoware_bridge.z_test_sub:main"
        ],
    },
)
