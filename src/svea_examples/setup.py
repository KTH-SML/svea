<<<<<<< HEAD
<<<<<<< HEAD
import os
from glob import glob
<<<<<<< HEAD
import xml.etree.ElementTree as ET
=======
from xml.etree.ElementTree import ET
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
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
        (f'lib/{name}', glob('scripts/*.py')),
        (os.path.join('share', name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').get('email'),
    description=package.find('description').text,
    license=package.find('license').text,
    tests_require=['pytest'],
=======
from setuptools import find_packages, setup
=======
>>>>>>> 1ea11f8 (Update package.xml and setup.py files for svea_core, svea_examples, and svea_localization to improve descriptions and maintainers' information)
import os
from glob import glob
from xml.etree.ElementTree import ET
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
        (f'lib/{name}', glob('scripts/*.py')),
        (os.path.join('share', name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').get('email'),
    description=package.find('description').text,
    license=package.find('license').text,
    tests_require=['pytest'],
<<<<<<< HEAD
    entry_points={
        'console_scripts': generate_console_scripts('scripts'),
        # 'console_scripts': [
        #     'svea_example = script.svea_example:main',
        #     'svea_example2 = script.svea_example2:main',
        # ],
    },
>>>>>>> 914c44e (update on 05/12/2025)
=======
>>>>>>> 5a67854 (Confirmed that symlink works)
)
