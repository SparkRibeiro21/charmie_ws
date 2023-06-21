##CHARMIE SIMULATOR

passos iniciais em cada tab do terminal:
- cd charmie_ws/
- source install/setup.bash

para usar joystick:
- conectar primeiro joystick via bluetooth ao pc
- ros2 launch charmie_bot joystick.launch.py
- L1 (normal) ou R1 (turbo) manter premido para acionar, quando solto robo para
- joystick esquerdo move X e Y
- joysitck  

iniciar ambiente gazebo:
- ambiente com robo - ros2 launch charmie_bot launch_sim.launch.py
- ambiente com robo e robocup world - ros2 launch charmie_bot launch_sim.launch.py world:=./src/charmie_bot/worlds/robocup_house.world

iniciar rviz com robo:
- rviz2 -d src/charmie_bot/config/main.rviz

iniciar slam_toolbox:
- ros2 launch slam_toolbox online_async_launch.py params_file:=./src/charmie_bot/config/mapper_params_online_async.yaml use_sim_time:=true

iniciar nav2 (para usar Goal Pose):

- ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
