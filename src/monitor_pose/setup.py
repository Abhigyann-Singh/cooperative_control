from setuptools import find_packages, setup

package_name = 'monitor_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhigyan',
    maintainer_email='abhigyan@todo.todo',
    description='Monitoring and control for rod-payload drones',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'monitor = monitor_pose.monitor:main',
            # Added your new keyboard control node here:
            'keyboard_control = monitor_pose.keyboard_rod_control:main',
        ],
    },
)