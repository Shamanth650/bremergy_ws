from setuptools import setup
import os
from glob import glob

package_name = 'driverless_intro'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Marvin Dufner',
    author_email='marvin.dufner@bremergy.de',
    description='An intro to autonomous driving in ros2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_visualizer = driverless_intro.cone_visualizer:main',
            'cat_visualizer = driverless_intro.cat_visualizer:main',
            'cat_position = driverless_intro.cat_position:main' 
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
)
