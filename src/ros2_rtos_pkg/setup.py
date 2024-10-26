from setuptools import setup

package_name = 'ros2_rtos_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teja',
    maintainer_email='TejasriKari@my.unt.edu',
    description='Turtle path planner in ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'turtle_path_planner = ros2_rtos_pkg.turtle_path_planner:main',
        ],
    },
)

