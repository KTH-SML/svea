import os
from glob import glob
from setuptools import find_packages, setup
import xml.etree.ElementTree as ET

package = ET.parse('package.xml').getroot()
name = package.find('name').text

setup(
    name=name,
    version='0.0.0',
    packages=find_packages(include=[name, f"{name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{name}']),
        (f'share/{name}', ['package.xml']),
        (f'share/{name}/launch', glob('launch/*.xml') + glob('launch/**/*.xml')),
        (f'share/{name}/launch', glob('launch/*.py')),
        (f'share/{name}/params', glob('params/*.yaml') + glob('params/**/*.yaml')),
        (f'lib/{name}', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').get('email'),
    description=package.find('description').text,
    license=package.find('license').text,
    tests_require=['pytest'],
)

