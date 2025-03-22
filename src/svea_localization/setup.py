from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('param/**/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='lic2@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuation_to_twist = script.actuation_to_twist:main',
            'gps_to_utm_relay = script.gps_to_utm_relay:main',
            'odom_to_path_relay = script.odom_to_path_relay:main',
            'plot_localization = script.plot_localization:main',
            'quat_to_euler_relay = script.quat_to_euler_relay:main',
            'rtk_manager = script.rtk_manager:main',
            'set_datum_node = script.set_datum_node:main',
        ],
    },
)

