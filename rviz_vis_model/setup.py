from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rviz_vis_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[          # link the files needed for visualization
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),       # add launch file
        ('share/' + package_name + '/models', ['models/Cornea.stl']),                                               # add eye and scalpel models
        ('share/' + package_name + '/models', ['models/Iris.stl']),
        ('share/' + package_name + '/models', ['models/Lense.stl']),
        ('share/' + package_name + '/models', ['models/Sclera.stl']),
        ('share/' + package_name + '/models', ['models/scalpel_cataract_v4.obj']),
        ('share/' + package_name + '/models', ['models/vis_eye_scalpel.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonaszuefle',
    maintainer_email='jonas.zuefle@student.kit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eye_model = rviz_vis_model.eye_model:main',
            'cut_model = rviz_vis_model.cut_model:main',
            'incision_points = rviz_vis_model.incision_points:main'
        ],
    },
)
