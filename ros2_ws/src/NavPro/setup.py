from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'navpro'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/navpro']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steve Hammer',
    maintainer_email='hammer@trinityrocks.com',
    description='NavPro: SLAM mapping, named waypoint saving, and voice-commanded navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navpro_node = navpro.navpro_node:main',
        ],
    },
)
