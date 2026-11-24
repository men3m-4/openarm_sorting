from glob import glob
import os

from setuptools import setup

package_name = 'openarm_sorting_bringup'


def tree(src_dir):
    """data_files entries that install src_dir recursively, keeping its layout."""
    entries = []
    for root, _dirs, files in os.walk(src_dir):
        if files:
            entries.append((os.path.join('share', package_name, root),
                            [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/coffee', glob('config/coffee/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='men3m',
    maintainer_email='muhammad.abdelmoniem4@gmail.com',
    description='Launch files, world, models and configuration of the OpenArm cup-sorting simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wait_for_controllers = openarm_sorting_bringup.wait_for_controllers:main',
        ],
    },
)
