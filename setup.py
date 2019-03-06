from setuptools import setup, find_packages

setup(
    name='nmea_navsat_driver',
    version='0.5.1',
    packages=find_packages(),
    py_modules=[],
    install_requires=['setuptools',
                      'pyserial',
                      'transforms3d',
                      'numpy'],
    author='Eric Perko',
    maintainer='Ed Venator',
    keywords=['ROS2'],
    description='Package to parse NMEA strings and publish a very simple GPS message.',
    license='BSD',
    entry_points={
        'console_scripts': ['nmea_serial_driver = nmea_navsat_driver.nodes.nmea_serial_driver:main',
                            'nmea_socket_driver = nmea_navsat_driver.nodes.nmea_socket_driver:main',
                            'nmea_topic_driver = nmea_navsat_driver.nodes.nmea_topic_driver:main',
                            'nmea_topic_serial_driver = nmea_navsat_driver.nodes.nmea_topic_serial_driver:main'],
    },
)
