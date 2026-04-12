import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mtt_bringup'

setup(
    # Other parameters ...
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],

    data_files=[
        # ... Other data files
        # Include all launch files.

        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='alex.chorel-campanozzi@ecn.forces.gc.ca',
    description='...',
    license='...',
    tests_require=['pytest'],

    entry_points={
    'console_scripts': [
        'odom_publisher_simul = mtt_bringup.odom_publisher_simul:main',
        'mtt_controller_interface = mtt_bringup.mtt_controller_interface:main',
        ],
    },   
)