from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'uwb'
submodules = os.path.join(package_name, 'submodules')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roman Eidelman, Artem Kondaratev',
    maintainer_email='reiv.dev@gmail.com, artemkondratev5@gmail.com',
    description='Package for streaming UWB coordinates',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb = uwb.uwb:main'
        ],
    },
)
