import os
from glob import glob
import xml.etree.ElementTree as ET
from setuptools import find_packages, setup

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
        (f'share/{name}/launch', glob('launch/*.py') + glob('launch/**/*.py')),
        (f'share/{name}/params', glob('params/*.yaml') + glob('params/**/*.yaml')),
        (f'share/{name}/meshes', glob('meshes/*')),
        (f'share/{name}/maps', glob('maps/*')),
        (f'share/{name}/urdf', glob('urdf/*')),
        (f'lib/{name}', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').get('email'),
    description=package.find('description').text,
    license=package.find('license').text,
)
