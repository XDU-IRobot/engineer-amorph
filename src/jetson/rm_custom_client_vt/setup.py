import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rm_custom_client_vt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunarifish',
    maintainer_email='messenger@lunariio.work',
    description='UDP H.265 video transport receiver for RoboMaster custom client',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'udp_video_receiver = rm_custom_client_vt.udp_video_receiver_node:main',
        ],
    },
)
