cmake_minimum_required(VERSION 3.8)
project(db_proxy_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AcquireTrafficArea.srv"
  "srv/ReleaseTrafficArea.srv"
  "srv/AddTrafficArea.srv"
  "srv/SqlQuery.srv"
  "srv/GenericQuery.srv"
  "srv/UpdateCarrier.srv"
  "srv/CarrierQuery.srv"
  "srv/RackQuery.srv"
  "msg/Carrier.msg"

  "srv/UpdateTask.srv"
  "msg/Task.msg"
  "msg/Tasks.msg"
  "srv/UpdateRack.srv"
  "msg/Rack.msg"
  "msg/Racks.msg"

  "msg/Work.msg"
  "msg/Works.msg"
  
  "msg/Location.msg"
  "msg/Locations.msg"

  "msg/Eqp.msg"
  "msg/Eqps.msg"

  "msg/EqpPort.msg"
  "srv/EqpSignalQuery.srv"
  "msg/EqpSignal.msg"

  "msg/Fetch.msg"
  "msg/Tables.msg"

  "msg/AGV.msg"
  "msg/AGVs.msg"
)

ament_package()
