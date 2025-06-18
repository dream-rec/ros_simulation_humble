from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_saver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Package for saving TurtleBot3 camera images',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = image_saver.image_saver_node:main',
            'interactive_image_saver = image_saver.interactive_image_saver:main',
        ],
    },
)
