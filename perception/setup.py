from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # pics - TODO

        # yolo models
        (os.path.join('share',package_name,'models'),
            glob('perception/models/*.pt')
        ),

        # config files
        (os.path.join('share',package_name,'config'),
            glob('config/*.yaml')
        )
        
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='ziga',
    maintainer_email='ziga.breznikar@student.um.si',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_pub = perception.publisher_node:main',
            'test_detekcija=perception.test:main',
            'test_service_client=perception.test_service_client:main',
            'perception_asp=perception.publisher_node_object:main'
        ],
    },
)
