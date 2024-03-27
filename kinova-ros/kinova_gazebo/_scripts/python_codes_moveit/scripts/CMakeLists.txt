add_executable(all_codes src/all.cpp)
add_executable(0currentpose_degrees src/0currentpose_degrees.cpp)
add_executable(0presentpose_xyz src/0presentpose_xyz.cpp)
add_executable(1jointspacegoal_forward src/1jointspacegoal_forward.cpp)
add_executable(2jointspacegoal_reverse src/2jointspacegoal_reverse.cpp)
add_executable(3posegoal src/3posegoal.cpp)
add_executable(4cartesianpath src/4cartesianpath.cpp)
add_executable(5pathConstraints src/5pathConstraints.cpp)
add_executable(try4 src/try4.cpp)



target_link_libraries (all_codes ${catkin_LIBRARIES})
target_link_libraries (0currentpose_degrees ${catkin_LIBRARIES})
target_link_libraries (0presentpose_xyz ${catkin_LIBRARIES})
target_link_libraries (1jointspacegoal_forward ${catkin_LIBRARIES})
target_link_libraries (2jointspacegoal_reverse ${catkin_LIBRARIES})
target_link_libraries (3posegoal ${catkin_LIBRARIES})
target_link_libraries (4cartesianpath ${catkin_LIBRARIES})
target_link_libraries (5pathConstraints ${catkin_LIBRARIES})
target_link_libraries (try4 ${catkin_LIBRARIES})



