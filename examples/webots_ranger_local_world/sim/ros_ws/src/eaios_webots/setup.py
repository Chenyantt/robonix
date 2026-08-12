from setuptools import setup
from glob import glob
import os

package_name = 'eaios_webots'

world_asset_files = []
for asset_root in ('worlds/objs', 'worlds/textures'):
    for root, _, files in os.walk(asset_root):
        for file in files:
            path = os.path.join(root, file)
            world_asset_files.append((os.path.join('share', package_name, root), [path]))

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.wbt') + glob('worlds/*.proto') + glob('worlds/*.dae')),

    ('share/' + package_name + '/resource', [
        'resource/ranger_mini_v3_webots.urdf',
        'resource/ros2_control.yml',
    ]),
] + world_asset_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robonix',
    maintainer_email='dev@robonix',
    description='Webots + ros2_control launch for Tiago-style worlds',
    license='MulanPSL-2.0',
    tests_require=['pytest'],
)
