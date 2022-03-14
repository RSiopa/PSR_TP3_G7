# PSR_TP3_G7
Third work from PSR

## How to run

1. In the terminal write the following:

`roslaunch p_spombinho_bringup game_bringup.launch`

This will launch the **gazebo** with the spwan of the various **cars**

2. Add the referee with the following command:

`rosrun th_referee th_referee`

3. Lastly, in another terminal write the following:

`roslaunch p_spombinho_bringup drivers_bringup.launch`

This will start the various drivers from each robot. This launch file has the following arguments:

*manual* and the private param *~image_flag*

*manual* - the user can use it todrive the red1 car or not.

*~image_flag* - Choose which cameras to see for the various robots


## Videos

Manual driving:
https://www.youtube.com/watch?v=xfK-qF10tmk

Autonomous driving:
https://www.youtube.com/watch?v=GP3YBwcxxp4
