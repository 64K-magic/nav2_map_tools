from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_center_web'

# Install entire web/ tree under share/control_center_web/web
web_files = []
for root, _dirs, files in os.walk('web'):
    for name in files:
        rel = os.path.join(root, name)
        dest_dir = os.path.join('share', package_name, os.path.dirname(rel))
        web_files.append((dest_dir, [rel]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        *web_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='64k-chen',
    maintainer_email='cdf_168@163.com',
    description='Web UI for RTK keepout figure drawing on tile maps',
    license='Apache-2.0',
)
