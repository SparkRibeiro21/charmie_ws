Written by Tiago Ribeiro


If you delete install and build folders from charmie_ws, you will need to do the following process: 

cd ~/charmie_ws/src/livox_ros_driver2/
./build_tr.sh humble

This will build the livox package with some Paths to variables it needs. However when you try to build the full workspace you will problable het this error:

--- stderr: livox_ros_driver2                             
failed to create symbolic link '/home/tiago/charmie_ws/build/livox_ros_driver2/ament_cmake_python/livox_ros_driver2/livox_ros_driver2' because existing path cannot be removed: Is a directory
gmake[2]: *** [CMakeFiles/ament_cmake_python_symlink_livox_ros_driver2.dir/build.make:70: CMakeFiles/ament_cmake_python_symlink_livox_ros_driver2] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:425: CMakeFiles/ament_cmake_python_symlink_livox_ros_driver2.dir/all] Error 2
gmake[1]: *** Waiting for unfinished jobs....
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< livox_ros_driver2 [2.31s, exited with code 2]

Summary: 0 packages finished [2.60s]
  1 package failed: livox_ros_driver2
  1 package had stderr output: livox_ros_driver2
tiago@tiago-ubuntu22:~/charmie_ws$ 

you will have to do the following command:

rm -rf ~/charmie_ws/build/livox_ros_driver2/ament_cmake_python/livox_ros_driver2/livox_ros_driver2

and now you can successfully:

colcon build --symlink-install --packages-select livox_ros_driver2
OR
colcon build --symlink-install

