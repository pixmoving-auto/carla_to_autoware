import os
from setuptools import setup
from glob import glob

package_name = 'my_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/*.launch')),
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
            "sub_node = my_bridge.sub:main",
            "carla_to_autoware_localization_node = my_bridge.carla_to_autoware_localization:main",
            "carla_to_autoware_vehicle_status_node = my_bridge.carla_to_autoware_vehicle_status:main",
            "odometry_to_posestamped_node = my_bridge.odometry_to_posestamped:main",
            "vehiclecmd_to_ackermanndrive = my_bridge.vehiclecmd_to_ackermanndrive:main"
        ],
    },
)
