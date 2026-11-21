from glob import glob
import os

from setuptools import setup

package_name = 'dc_robot'


def share(subdir, pattern='*'):
    """Install every file matching `subdir/pattern` into share/<pkg>/<subdir>."""
    files = [f for f in glob(os.path.join(subdir, pattern)) if os.path.isfile(f)]
    return (os.path.join('share', package_name, subdir), files)


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        share('launch', '*.launch.py'),
        share('urdf', '*.xacro'),
        share('worlds', '*.world'),
        share('rviz', '*.rviz'),
        share('config', '*.yaml'),
        share('maps'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keval Visaria',
    maintainer_email='visariakeval@gmail.com',
    description='dc_robot (data-centre robot): URDF, Gazebo worlds, SLAM and Nav2 configs.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'depth_cloud_relay = dc_robot.depth_cloud_relay:main',
    ]},
)
