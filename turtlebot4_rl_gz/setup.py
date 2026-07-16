from setuptools import find_packages, setup

package_name = 'turtlebot4_rl_gz'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'turtlebot4_rl_core'],
    zip_safe=True,
    maintainer='xiaodudu',
    maintainer_email='1412822254@qq.com',
    description='Gazebo Harmonic TurtleBot 4 RL world adapter.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
