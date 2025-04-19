from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anscer_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (
            f'share/{package_name}', ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))
        ),
        (
            os.path.join('share', package_name, 'action'),
            glob(os.path.join('action', '*.action'))
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bonnybk',
    maintainer_email='bonnybabukachappilly@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
