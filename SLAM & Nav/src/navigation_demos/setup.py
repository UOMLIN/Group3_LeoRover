from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        #.srv file
        (os.path.join('share', package_name, 'srv'), glob(os.path.join('srv', '*.srv'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adarshkaran',
    maintainer_email='adarshkaran@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'navigation_server = navigation_demos.navigation_server:main',
            # 'navigation_client = navigation_demos.navigation_client:main',
            'navigation_server_copy = navigation_demos.navigation_server_copy:main',
            'navigation_client_copy = navigation_demos.navigation_client_copy:main',
            'nav_to_pose_commander = navigation_demos.nav_to_pose_commander:main'
            
            
        ],
    },
)