## RQT Node Graph
> <img src="../images/rosgraph.png" style="vertical-align:middle; padding:25px 25px 25px 25px" width="1000">
## Explanation
# Nodes we created:
> #Controller
> - This node is suscribed to the /odom, /camera/color/image_raw and /map topics to get information
> - This node then calculates the bot's path usign A star algorithm explained <a href="https://github.com/sid-5/ROS_VICHESTA/blob/master/docs/problem_statement.md">here</a> to calculate the path of the bot and publishes the required information to /cmd_vel to localise the bot
> - When the path reaches certain way-points in the mao this node sends the information form the suscribed node to the servers for performing ball detection(explained <a href="https://github.com/sid-5/ROS_VICHESTA/blob/master/docs/workflow1.md">here</a>), aruco id and box colour detection(explained <a href="https://github.com/sid-5/ROS_VICHESTA/blob/master/docs/workflow1.md">here</a>) and door colour detection(explained <a href="https://github.com/sid-5/ROS_VICHESTA/blob/master/docs/workflow1.md">here</a>)

#Servive files we created:
>Ball_server
>Door_server
>Aruco_color_server
