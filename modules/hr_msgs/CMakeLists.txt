##
## Copyright (C) 2017-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##
cmake_minimum_required(VERSION 2.8.3)
project(hr_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  dynamic_reconfigure
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Assign.msg
  AudioStream.msg
  BlinkCycle.msg
  Body.msg
  ChatMessage.msg
  ChatResponse.msg
  ChatResponses.msg
  CurrentFrame.msg
  Doa.msg
  Event.msg
  EventMessage.msg
  FSShapekey.msg
  FSShapekeys.msg
  FSValues.msg
  Face.msg
  FaceEvent.msg
  Faces.msg
  FacialActionUnit.msg
  Feature.msg
  Features.msg
  Forget.msg
  ForgetAll.msg
  Gesture.msg
  Gestures.msg
  GetAPIVersion.msg
  Hand.msg
  HeadFun.msg
  Luminance.msg
  MakeFaceExpr.msg
  ManyEarsTrackedAudioSource.msg
  Motion.msg
  MotionSignal.msg
  Motions.msg
  MotorCommand.msg
  MotorState.msg
  MotorStateList.msg
  Object.msg
  Objects.msg
  People.msg
  PeopleFace.msg
  PeopleFaces.msg
  Person.msg
  PiFace.msg
  PiVisionFace.msg
  PiVisionFaces.msg
  PlayAnimation.msg
  Point2D.msg
  Point2DArray.msg
  PointHead.msg
  SaccadeCycle.msg
  OnsetMessage.msg
  SalientPoint.msg
  SetAnimation.msg
  SetExpression.msg
  SetGestureClip.msg
  SomaState.msg
  SomaStates.msg
  SoulTalkChat.msg
  SourceInfo.msg
  SourceInfoWithCovariance.msg
  Status.msg
  Stroke.msg
  Strokes.msg
  TTS.msg
  TTSItem.msg
  Target.msg
  TargetPosture.msg
  Tracker.msg
  Trackers.msg
  VADMessage.msg
  Viseme.msg
  Visemes.msg
  Vision.msg
  audiodata.msg
  f_id.msg
  facebox.msg
  faces.msg
  faces_ids.msg
  pau.msg
  targets.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  AgentChat.srv
  AgentFeedback.srv
  AgentRegister.srv
  AgentUnregister.srv
  AnimationLength.srv
  ChatService.srv
  ChatSession.srv
  ConfigurableNodes.srv
  Current.srv
  Emotion.srv
  EyeState.srv
  FaceId.srv
  FaceLandmarks.srv
  GetAnimationLength.srv
  GetInteger.srv
  GetIntent.srv
  GetMode.srv
  GetParam.srv
  Json.srv
  Load.srv
  LoadPerformance.srv
  MotionManagement.srv
  MotorStates.srv
  MotorValues.srv
  MotorValuesBool.srv
  NodeAction.srv
  NodeConfiguration.srv
  NodeDescription.srv
  Pause.srv
  PerformanceCommand.srv
  ResetMotors.srv
  Resume.srv
  Run.srv
  RunByName.srv
  SaveFace.srv
  SetActuatorsControl.srv
  SetMode.srv
  SetParam.srv
  SetProperties.srv
  SetScene.srv
  Stop.srv
  StringArray.srv
  StringTrigger.srv
  TTSData.srv
  TTSLength.srv
  TTSTrigger.srv
  UpdateExpressions.srv
  UpdateMotors.srv
  ValidFaceExprs.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder

generate_dynamic_reconfigure_options(
  cfg/Parallax.cfg
  cfg/Keepalive.cfg
  cfg/ExternalAnimation.cfg
  cfg/Animations.cfg
  cfg/ExternalArmsPose.cfg
  cfg/Expressions.cfg
  cfg/Visemes.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hr_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(hr_msgs
#   src/${PROJECT_NAME}/hr_msgs.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(hr_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(hr_msgs_node src/hr_msgs_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hr_msgs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(hr_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS hr_msgs hr_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hr_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
