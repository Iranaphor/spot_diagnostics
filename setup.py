from setuptools import setup
from glob import glob
import os

package_name = 'spot_diagnostics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml'])
    ],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='Package for managing diagnostics for spot',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt.py = spot_diagnostics.mqtt:main'
        ],
    },
)
