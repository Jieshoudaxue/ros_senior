cmake_minimum_required(VERSION 3.0.2)
project(ar_track_alvar)

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        cv_bridge
        dynamic_reconfigure
        geometry_msgs
        image_transport
        message_generation
        pcl_conversions
        pcl_ros
        resource_retriever
        roscpp
        sensor_msgs
        std_msgs
        tf
        tf2
        visualization_msgs)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)

# dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Params.cfg)

set(MSG_FILES AlvarMarker.msg AlvarMarkers.msg)
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
        cv_bridge
        dynamic_reconfigure
        geometry_msgs
        image_transport
        message_runtime
        pcl_conversions
        pcl_ros
        resource_retriever
        roscpp
        sensor_msgs
        std_msgs
        tf
        tf2
        visualization_msgs
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${TinyXML_INCLUDE_DIRS}

)

add_library(${PROJECT_NAME}
    src/Bitset.cpp
    src/Camera.cpp
    src/CaptureDevice.cpp
    src/CaptureFactory.cpp
    src/CaptureFactory_unix.cpp
    src/ConnectedComponents.cpp
    src/CvTestbed.cpp
    src/DirectoryIterator.cpp
    src/DirectoryIterator_unix.cpp
    src/Draw.cpp
    src/FileFormatUtils.cpp
    src/Filter.cpp
    src/Kalman.cpp
    src/Line.cpp
    src/Marker.cpp
    src/MarkerDetector.cpp
    src/MultiMarker.cpp
    src/MultiMarkerBundle.cpp
    src/MultiMarkerInitializer.cpp
    src/Mutex.cpp
    src/Mutex_unix.cpp
    src/Optimization.cpp
    src/Plugin.cpp
    src/Plugin_unix.cpp
    src/Pose.cpp
    src/Rotation.cpp
    src/Threads.cpp
    src/Threads_unix.cpp
    src/Util.cpp
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Kinect filtering code
add_library(kinect_filtering src/kinect_filtering.cpp)
target_link_libraries(kinect_filtering ${catkin_LIBRARIES})
add_dependencies(kinect_filtering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(medianFilter src/medianFilter.cpp)
target_link_libraries(medianFilter ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(medianFilter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(individualMarkers nodes/IndividualMarkers.cpp)
target_link_libraries(individualMarkers ${PROJECT_NAME} kinect_filtering ${catkin_LIBRARIES})
add_dependencies(individualMarkers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(individualMarkersNoKinect nodes/IndividualMarkersNoKinect.cpp)
target_link_libraries(individualMarkersNoKinect ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(individualMarkersNoKinect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(trainMarkerBundle nodes/TrainMarkerBundle.cpp)
target_link_libraries(trainMarkerBundle ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(trainMarkerBundle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(findMarkerBundles nodes/FindMarkerBundles.cpp)
target_link_libraries(findMarkerBundles ${PROJECT_NAME} kinect_filtering medianFilter ${catkin_LIBRARIES})
add_dependencies(findMarkerBundles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(findMarkerBundlesNoKinect nodes/FindMarkerBundlesNoKinect.cpp)
target_link_libraries(findMarkerBundlesNoKinect ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(findMarkerBundlesNoKinect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(createMarker src/SampleMarkerCreator.cpp)
target_link_libraries(createMarker ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(createMarker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} createMarker findMarkerBundles findMarkerBundlesNoKinect individualMarkers individualMarkersNoKinect kinect_filtering medianFilter trainMarkerBundle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY bundles launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  file(GLOB LAUNCH_FILES launch/*.launch test/*.test)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE} USE_TEST_DEPENDENCIES)
  endforeach()
endif()
