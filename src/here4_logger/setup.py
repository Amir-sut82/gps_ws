from glob import glob

from setuptools import find_packages, setup


setup(
    name='here4_logger',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/here4_logger']),
        ('share/here4_logger', ['package.xml']),
        ('share/here4_logger/launch', glob('launch/*.launch.py')),
        ('share/here4_logger/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GPS workspace maintainer',
    maintainer_email='maintainer@example.com',
    description='PX4 GPS and EKF position monitors with CSV logs for each armed session',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'gps_logger = here4_logger.node:main',
        'ekf_logger = here4_logger.ekf_node:main',
    ]},
)
