from setuptools import setup
import os
from glob import glob

package_name = 'glr_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Include RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # Include models
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
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
