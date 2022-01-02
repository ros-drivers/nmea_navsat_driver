cmake_minimum_required(VERSION 3.0.2)
project(nmea_navsat_driver)

if (NOT CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
  )
endif()
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
    catkin_virtualenv
  )
endif()

catkin_python_setup()
catkin_package()

install(PROGRAMS
   scripts/nmea_serial_driver
   scripts/nmea_socket_driver
   scripts/nmea_topic_driver
   scripts/nmea_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

install(FILES
   LICENSE.txt
   test/requirements.txt
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
  )

  catkin_add_nosetests(
    test/test_driver.py
    DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_virtualenv
  )

  catkin_install_python(
    PROGRAMS
      test/test_driver.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  find_package(roslint REQUIRED)
  roslint_python()
  roslint_add_test()
endif()
