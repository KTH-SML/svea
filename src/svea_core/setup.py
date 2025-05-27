from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (f'share/{package_name}/params', glob('params/*.yaml')),
        (f'share/{package_name}/maps', glob('maps/*')),
        (f'lib/{package_name}', glob('scripts/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools','pyyaml'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='lic2@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
