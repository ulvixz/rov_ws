import os
from glob import glob
from setuptools import find_packages, setup


package_name = "rov_autonomy"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    ),

    (
        os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')
    ),

    (
        os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')
    ),
],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kaplan",
    maintainer_email="kaplan@example.com",
    description="ROS 2 autonomy package for the ROV.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
    'console_scripts': [
        'twist_to_manual_control = rov_autonomy.twist_to_manual_control:main',
        'joy_trigger_mixer = rov_autonomy.joy_trigger_mixer:main',
    ],
},
)
