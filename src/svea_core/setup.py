from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_core'

def generate_console_scripts(script_dir=f'src/{package_name}/script'):
    console_scripts = []
    for path in glob(os.path.join(script_dir, '*.py')):
        filename = os.path.basename(path)
        module_name = filename[:-3]
        if module_name == '__init__':
            continue
        entry = f'{module_name} = script.{module_name}:main'
        console_scripts.append(entry)
    return console_scripts

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.xml')),
        (f'share/{package_name}/params', glob('params/*.yaml')),
        (f'share/{package_name}/maps', glob('maps/*')),
        (f'lib/{package_name}', glob('scripts/*.py')),  # Install raw executables to lib/{package_name}
    ],
    install_requires=['setuptools','pyyaml'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='lic2@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
