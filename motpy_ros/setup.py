import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'motpy_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    author='Ar-Ray-code',
    author_email="ray255ar@gmail.com",
    maintainer='user',
    maintainer_email="user@todo.todo",
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO: Package description.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'darknet_tracking = '+ package_name + '.darknet_tracking:ros_main',
            'face_detect = '+ package_name + '.face_detect:ros_main',
            'mosaic_bbox = '+ package_name + '.mosaic_bbox:ros_main',
        ],
    },
)