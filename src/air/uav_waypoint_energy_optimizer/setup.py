from glob import glob
from setuptools import find_packages, setup

package_name = 'uav_waypoint_energy_optimizer'

setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml', 'main.sh']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=[
        'setuptools>=30.3.0,<80',
    ],

    zip_safe=True,

    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',

    description=(
        'Energy-model based multi-waypoint visit-order optimization '
        'and cruise mission for AirSim.'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'waypoint_energy_optimizer = '
            'uav_waypoint_energy_optimizer.waypoint_cruise_node:main',
        ],
    },
)
