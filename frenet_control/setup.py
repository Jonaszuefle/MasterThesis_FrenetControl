from setuptools import find_packages, setup

package_name = 'frenet_control'

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
            'asdf_pi_control_old = frenet_control.pi_control:main',
            'pi_control = frenet_control.pi_controller:main',
            'mpc_control = frenet_control.mpc_controller:main',
        ],
    },
)
