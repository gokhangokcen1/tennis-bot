from setuptools import find_packages, setup
from glob import glob

package_name = 'tennis_bot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gokhangokcen1',
    maintainer_email='gokhangokcenn@gmail.com',
    description='Nav2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "collection_planner_node = tennis_bot_navigation.collection_planner_node:main",
            "cmd_vel_relay_node = tennis_bot_navigation.cmd_vel_relay_node:main",
        ],
    },

)
