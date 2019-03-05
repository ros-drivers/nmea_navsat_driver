from setuptools import setup, find_packages

setup(
    name='libnmea_navsat_driver',
    version='0.5.1',
    packages=find_packages(),
    py_modules=[],
    install_requires=['setuptools',
                      'pyserial'],
    author='Eric Perko',
    maintainer='Ed Venator',
    keywords=['ROS2'],
    description='Package to parse NMEA strings and publish a very simple GPS message.',
    license='BSD',
    entry_points={
        'console_scripts': [],
    },
)
