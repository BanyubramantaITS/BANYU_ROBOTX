from setuptools import find_packages, setup

package_name = 'banyu_robocommand'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gryd',
    maintainer_email='jalucahyoseno@gmail.com',
    description='RoboCommand (RobotX 2026) MQTT/protobuf bridge for the OCS.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rc_bridge = banyu_robocommand.rc_bridge:main',
        ],
    },
)
