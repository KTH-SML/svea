from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_core'

# packages=[
#     'svea',
#     'svea.controllers',
#     'svea.models',
#     'svea.simulators',
#     'svea.svea_managers',
#     'svea.tests',
# ]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # package_dir= {'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('**/*.yaml')),
    ],
    # py_modules = [
    #     'script.state_publisher',
    # ],
    install_requires=['setuptools','pyyaml'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='lic2@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = script.state_publisher:main',
        ],
    },
)
