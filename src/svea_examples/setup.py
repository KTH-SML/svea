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
import os
from glob import glob

package_name = 'svea_examples'

def generate_console_scripts(script_dir):
    console_scripts = []
    for path in glob(os.path.join(script_dir, '*.py')):
        filename = os.path.basename(path)
        module_name = filename[:-3]
        if module_name == '__init__':
            continue
        entry = f'{module_name} = scripts.{module_name}:main'
        console_scripts.append(entry)
    return console_scripts

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (f'share/{package_name}/launch', glob('launch/*.xml')),
        (f'lib/{package_name}', glob('scripts/*.py')),  # Install raw executables to lib/{package_name}
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='lic2@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': generate_console_scripts('scripts'),
        # 'console_scripts': [
        #     'svea_example = script.svea_example:main',
        #     'svea_example2 = script.svea_example2:main',
        # ],
    },
>>>>>>> 914c44e (update on 05/12/2025)
)
