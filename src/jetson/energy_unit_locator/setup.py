from setuptools import find_packages, setup

package_name = 'energy_unit_locator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets', ['assets/251120_epoch335.pt']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='s',
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
            'locator = energy_unit_locator.locator_node:main',
            'locator_v2 = energy_unit_locator.locator_node_v2:main',
            'yolo_test = energy_unit_locator.yolo_test_node:main',
        ],
    },
)
