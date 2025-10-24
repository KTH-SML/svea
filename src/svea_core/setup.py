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
        (f'share/{name}/launch', glob('launch/*.xml')),
        (os.path.join('share', name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (f'share/{name}/params', glob('params/*.yaml')),
        (f'share/{name}/maps', glob('maps/*')),
        (f'lib/{name}', glob('scripts/*.py')),
        (f'share/{name}/urdf', glob('urdf/*')),
        (os.path.join('share', name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', name, 'util'), glob('util/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').get('email'),
    description=package.find('description').text,
    license=package.find('license').text,
)
