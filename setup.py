from glob import glob
import os
from setuptools import setup

PACKAGE_NAME = "nmea_navsat_driver"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='0.5.1',
    packages=["scripts", "libnmea_navsat_driver"],
    data_files=[
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    package_dir={'': 'src',
                 "scripts": "scripts"},
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyserial',
                      'transforms3d',
                      'numpy',
                      'pyyaml'],
    author='Eric Perko',
    maintainer='Ed Venator',
    keywords=['ROS2'],
    description='Package to parse NMEA strings and publish a very simple GPS message.',
    license='BSD',
    entry_points={
        'console_scripts': ['nmea_serial_driver = scripts.nmea_serial_driver:main',
                            'nmea_socket_driver = scripts.nmea_socket_driver:main',
                            'nmea_topic_driver = scripts.nmea_topic_driver:main',
                            'nmea_topic_serial_reader = scripts.nmea_topic_serial_reader:main'],
    }
)
