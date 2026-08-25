from setuptools import setup
import os
from glob import glob

package_name = 'glr_description'


def list_files(pattern):
    return [path for path in glob(pattern, recursive=True) if os.path.isfile(path)]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), list_files('launch/*.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), list_files('urdf/**/*')),
        # Include RViz configs
        (os.path.join('share', package_name, 'rviz'), list_files('rviz/*')),
        # Include models
        (os.path.join('share', package_name, 'models'), list_files('models/**/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jardel Dyonisio',
    maintainer_email='jardel.dyonisio@hotmail.com',
    description='GLR robot description package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = glr_description.odom:main',
            'teleop_keyboard = glr_description.teleop_keyboard:main',
        ],
    },
)
