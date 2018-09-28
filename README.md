# H-GM-Humanoid-Robot-Real-Time-Trajectory-Manipulator
This program was created by me in order to allow to visualize and manipulate the behaviour of the part of a humanoid robot in real-time.

It was coded in C++ and I had used the OpenGL in order to draw the 3D and 2D objects on the scene.

Primarily, I had used only the GLFW library as a basis in which I could develop the program, since it provides a main loop hotspot to code interactive programs, the rest was accomplished from scratch. The geometry of the humanoid robot parts were drawn with GL draw calls regarding spheres and lines. Moreover, I had deployed a B-Spline render function in order to draw the foot trajectory. The center of mass, depicted with the blue line was drawn to help keeping track of the behaviour of the robot. I had developed this interface with a parallel architecture that allow to send and print several messages in different terminals, in order to be able to send several messages in real-time to the user.

Primarily, this program allows to visualize the GAIT (way of the movement of the limbs) parameters, accelerometer parameters, frame execution time, data transmission time.

It allows to control the servo mapping in real-time, which means that it is possible to determine the rotation range and direction for each one of them. Besides that, it also allows to control parameters regarding the GAIT being performed through a parametric curve configuration.

The program communicates in real-time with an Arduino based architecture that serves as a com controller to the main servo controller installed in the robot.
