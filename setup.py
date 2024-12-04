from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'data_collection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polyblank',
    maintainer_email='achermann@campus.tu-berlin.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant_position_publisher = data_collection_pkg.plant_position_publisher:main',
            'plant_velocity_publisher = data_collection_pkg.plant_velocity_publisher:main',
            'plant_data_subscriber = data_collection_pkg.plat_data_subscriber:main',
        ],
    },
)
