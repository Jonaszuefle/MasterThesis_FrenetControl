from setuptools import find_packages, setup

package_name = 'ros_bag_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irs',
    maintainer_email='uspqk@student.kit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_logger = ros_bag_recorder.data_logger:main',
            'manual_recorder = ros_bag_recorder.manual_recorder:main',
        ],
    },
)
