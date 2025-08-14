from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sonar_camera_reconstruction_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivana Collado',
    maintainer_email='icollado@stevens.edu',
    description='sonar_camera_reconstruction package',
    license='MIT',
    extras_require={
        'test':['pytest'],
    },
    entry_points={
        'console_scripts': [
            'merge_node = sonar_camera_reconstruction_pkg.merge_node:main'
        ],
    },
)
