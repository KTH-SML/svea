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
<<<<<<< HEAD
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
<<<<<<< HEAD
        # Script files
        (f'lib/{package_name}', glob('scripts/*.py')),  # Install raw executables to lib/{package_name}
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob(f'src/{package_name}/launch/*.xml')),
        # Params/config files
        (os.path.join('share', package_name, 'params'), glob(f'src/{package_name}/params/*.yaml')),
        # Maps
        (os.path.join('share', package_name, 'maps'), glob(f'src/{package_name}/maps/*')),
=======
        (f'share/{package_name}/launch', glob('launch/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (f'share/{package_name}/params', glob('params/*.yaml')),
        (f'share/{package_name}/maps', glob('maps/*')),
        (f'lib/{package_name}', glob('scripts/*.py')),
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 48e60ec (Confirmed that symlink works)
=======
        (os.path.join('share', package_name), glob('urdf/*')),
=======
        (f'share/{package_name}/urdf', glob('urdf/*')),
>>>>>>> 579c547 (multi model displacement with namespace in foxglove fixed)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
<<<<<<< HEAD
>>>>>>> 710b561 (Teleop control in simulation with teleop_twist_keyboard added)
=======
        (os.path.join('share', package_name, 'util'), glob('util/*.sh')),
>>>>>>> f6b073e (test version 1.0 update)
=======
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
>>>>>>> 10ada71 (Update package.xml and setup.py files for svea_core, svea_examples, and svea_localization to improve descriptions and maintainers' information)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').get('email'),
    description=package.find('description').text,
    license=package.find('license').text,
)
