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
        ('share/' + package_name + '/models', ['resource/251120_epoch335.pt', 'resource/251120_epoch335.engine']),
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
            'locator = amorph_vision.locator_node:main',
            'locator_v2 = amorph_vision.locator_node_v2:main',
            'yolo_test = amorph_vision.yolo_test_node:main',
            'export_engine = amorph_vision.engine_exporter:main',
        ],
    },
)
