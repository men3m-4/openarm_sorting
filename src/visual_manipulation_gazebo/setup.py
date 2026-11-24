from setuptools import setup

package_name = 'visual_manipulation_gazebo'

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
    description='Gazebo-only integration layer: ground truth tools and the optional grasp assist',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_assist = visual_manipulation_gazebo.grasp_assist:main',
            'check_portafilter = visual_manipulation_gazebo.check_portafilter:entry',
        ],
    },
)
