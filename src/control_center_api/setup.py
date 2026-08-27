from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_center_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'pydantic',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='64k-chen',
    maintainer_email='cdf_168@163.com',
    description='Python API for RTK keepout figure edit / convert / store',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_center_api = control_center_api.main:main',
        ],
    },
)
