from setuptools import find_packages, setup

package_name = 'pid_plotter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roma',
    maintainer_email='reiv.dev@gmail.com',
    description='This package show plot of PID data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_plotter = pid_plotter.pid_plotter:main'
        ],
    },
)
