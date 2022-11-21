from glob import glob
import os
from setuptools import setup

PACKAGE_NAME = "nmea_navsat_driver"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='2.0.0',
    packages=["libnmea_navsat_driver", "libnmea_navsat_driver.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    package_dir={'': 'src', },
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyserial',
                      'numpy',
                      'pyyaml'],
    author='Eric Perko',
    maintainer='Ed Venator',
    keywords=['ROS2'],
    description='Package to parse NMEA strings and publish a very simple GPS message.',
    license='BSD',
    entry_points={
        'console_scripts': ['nmea_serial_driver = libnmea_navsat_driver.nodes.nmea_serial_driver:main',
                            'nmea_socket_driver = libnmea_navsat_driver.nodes.nmea_socket_driver:main',
                            'nmea_tcpclient_driver = libnmea_navsat_driver.nodes.nmea_tcpclient_driver:main',
                            'nmea_topic_driver = libnmea_navsat_driver.nodes.nmea_topic_driver:main',
                            'nmea_topic_serial_reader = libnmea_navsat_driver.nodes.nmea_topic_serial_reader:main'],
    }
)
