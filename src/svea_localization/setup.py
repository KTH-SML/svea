from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_localization'

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
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('params/**/*.yaml')),
        # Install raw executables to lib/{package_name}
        (f'lib/{package_name}', glob('scripts/*.py')),  
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
)

