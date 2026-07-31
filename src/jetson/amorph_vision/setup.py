from setuptools import find_packages, setup

package_name = 'amorph_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'resource/best.pt',
            'resource/energy_unit.ply',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunarifish',
    maintainer_email='messenger@lunariio.work',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'eu_locator_v3_node = amorph_vision.eu_locator_v3_node:main',
        ],
    },
)
