from glob import glob

from setuptools import setup

package_name = 'openarm_sorting_description'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/presets', glob('presets/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='men3m',
    maintainer_email='muhammad.abdelmoniem4@gmail.com',
    description='Gazebo variant of the OpenArm v2.0 robot description for the cup-sorting simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
)
