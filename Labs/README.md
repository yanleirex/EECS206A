# Labs FAQ and remarks

## Lab1 and Lab2
These two labs contain a simple introduction to ROS, including ROS publisher, subscriber, node and some stuff about python. It should be quite easy if you follow the ROS official tutorials.

One small tip is presented for Part 4 in Lab 2. In order to create multiple turtles in the simulation. I would recommend to use ` rosservice`, such as `rosservice call spawn 1 1 0 turtle`, where three double represent the x, y postion and the rotational angle, respectively.

## Lab3
Please refer to quaternion representation and transformation matrix involving twist representation, which is strongly recommended.

If you have difficulty with Part 1.4, I would recommend you to use `rosrun tf view_frames`, which generates a list of frames in the terminal and summary them up in a pdf file. As you might noticed, the system has more than thirty frames. It's fine to choose any two of them to finish part 1.4.
