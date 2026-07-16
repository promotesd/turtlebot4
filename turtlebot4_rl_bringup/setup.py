from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlebot4_rl_bringup'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'PyYAML>=6',
        'turtlebot4_rl_core',
        'turtlebot4_rl_mock',
        'turtlebot4_dqn',
    ],
    zip_safe=True,
    maintainer='xiaodudu',
    maintainer_email='1412822254@qq.com',
    description='TurtleBot 4 RL bringup and examples.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_mock = turtlebot4_rl_bringup.train_mock:main',
            'train_rl = turtlebot4_rl_bringup.train_mock:main',
        ]
    },
)
