from setuptools import find_packages, setup

package_name = 'turtlebot4_rl_ros'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy>=1.24', 'turtlebot4_rl_core'],
    zip_safe=True,
    maintainer='xiaodudu',
    maintainer_email='1412822254@qq.com',
    description='ROS 2 TurtleBot 4 RL robot adapter.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
