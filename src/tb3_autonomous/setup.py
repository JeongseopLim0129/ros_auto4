from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_autonomous'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TurtleBot3 자율주행 패키지 (Custom Planner + Nav2 일부 사용)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── 직접 만든 노드 ──────────────────────────────────────
            'global_planner   = tb3_autonomous.global_planner_node:main',
            'local_controller = tb3_autonomous.local_controller_node:main',
            'goal_manager     = tb3_autonomous.goal_manager_node:main',
        ],
    },
)
