from setuptools import find_packages, setup

package_name = 'master_slave_control'

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
    maintainer='varga',
    maintainer_email='varga@fzi.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automatic_incision = master_slave_control.automatic_incision:main',
            'master2slave = master_slave_control.master2slave:main',
        ],
    },
)
