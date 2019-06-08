cmake_minimum_required(VERSION 2.8.3)
project(p2os_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  TCMState.msg
  AIO.msg
  ArmState.msg
  BatteryState.msg
  DIO.msg
  GripperState.msg
  GripState.msg
  LiftState.msg
  MotorState.msg
  PTZState.msg
  SonarArray.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs
)