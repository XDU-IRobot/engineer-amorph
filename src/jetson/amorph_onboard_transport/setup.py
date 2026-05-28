from setuptools import find_packages, setup

package_name = 'amorph_onboard_transport'

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
    maintainer='lunarifish',
    maintainer_email='lunarifish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'can_eu_broadcaster = amorph_onboard_transport.can_eu_broadcaster:main',
            'two_point_range_broadcaster = amorph_onboard_transport.2point_range_broadcaster:main',
        ],
    },
)
