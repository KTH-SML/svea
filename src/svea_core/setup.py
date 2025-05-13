from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
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
        (f'share/{package_name}/params', glob('params/*.yaml')),
        (f'share/{package_name}/maps', glob('maps/*')),
        (f'lib/{package_name}', glob('scripts/*.py')),
>>>>>>> 48e60ec (Confirmed that symlink works)
    ],
    install_requires=['setuptools','pyyaml'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='lic2@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
