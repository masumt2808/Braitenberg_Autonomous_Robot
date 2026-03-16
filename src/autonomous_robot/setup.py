from setuptools import find_packages, setup

package_name = 'autonomous_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.edu',
    description='Autonomous cleaning robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = autonomous_robot.obstacle_avoidance_node:main',
            'teleop_node = autonomous_robot.teleop_node:main',
        ],
    },
)
