cmake_minimum_required(VERSION 3.22)
project(mycobot_320pi_gz)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  scripts/test.publisher.py
  DESTINATION lib/${PROJECT_NAME}
)