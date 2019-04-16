from setuptools import setup, find_packages

setup(
    name='nmea_navsat_driver',
    version='0.5.1',
    packages=find_packages(),
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
                            'nmea_topic_serial_driver = scripts.nmea_topic_serial_driver:main'],
    },
)
