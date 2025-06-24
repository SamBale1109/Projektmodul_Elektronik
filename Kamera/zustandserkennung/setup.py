from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zustandserkennung'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'models'),glob('models/*')),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukashebio',
    maintainer_email='sambale@hebio.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "boiling_detection_node = zustandserkennung.boiling_detection_node:main",
            "camera_publisher = zustandserkennung.camera_publisher:main"
        ],
    },
)
