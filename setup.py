from setuptools import setup

setup(
    name='libnmea_navsat_driver',
    version='0.5.1',
    packages=["scripts", "libnmea_navsat_driver"],
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
