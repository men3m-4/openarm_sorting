from setuptools import setup

package_name = 'visual_manipulation_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='men3m',
    maintainer_email='muhammad.abdelmoniem4@gmail.com',
    description='Robot-independent MoveIt 2 / gripper / planning-scene adapters',
    license='Apache-2.0',
    tests_require=['pytest'],
)
