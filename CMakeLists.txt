# Impostazioni minime per utilizzare Catkin
cmake_minimum_required(VERSION 2.8.3)
project(second_project)

# Aggiungi dipendenze per il pacchetto Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  message_generation
)

generate_messages(
DEPENDENCIES
std_msgs
)

# Declara il pacchetto
catkin_package(
  
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf message_runtime
)

# Specifica dove si trovano i tuoi file sorgente
add_executable(odomtotf src/odomtotf.cpp)


# Collega le librerie richieste per ogni nodo
target_link_libraries(odomtotf ${catkin_LIBRARIES})

# Aggiungi le directory dove si trovano gli header
include_directories(
  ${catkin_INCLUDE_DIRS}
)



