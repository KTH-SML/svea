from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_core'

packages=[
    'svea_core',
    'svea_core.controllers',
    'svea_core.models',
    'svea_core.simulators',
    'svea_core.tests',
    'svea_core.script',
]

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    # packages=find_packages(exclude=['test']),
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
        (os.path.join('share', package_name, 'config'), glob('**/*.yaml')),
    ],
    install_requires=['setuptools','pyyaml'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='lic2@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = svea_core.script.state_publisher:main',
            'lli_test = svea_core.script.lli_test:main',
            'plot_map = svea_core.script.plot_map:main',
            'save_map = svea_core.script.save_map:main',
            'sim_svea = svea_core.simulators.sim_SVEA:main',
            'sim_lidar = svea_core.simulators.sim_lidar:main',
            # 'pure_pursuit = svea_core.script.pure_pursuit:main',
        ],
    },
)
