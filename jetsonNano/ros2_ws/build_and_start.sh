rm -rf install log build/*
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G "Unix Makefiles"
source install/setup.bash
ros2 launch geicar_start_jetson geicar.jetson.launch.py
