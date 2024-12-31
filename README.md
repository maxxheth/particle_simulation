# Particle Simulation
Particle Simulation using OpenGL and GLFW.

![Simulation Result](images/particle_gif.gif)

# Install dependencies
```bash
$ sudo apt-get install libglfw3-dev
$ sudo apt-get install libglew-dev
$ sudo apt-get install freeglut3-dev
$ sudo apt-get install libtbb-dev
$ sudo apt-get install libeigen3-dev
```

# Clone the package into catkin_ws/src
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/jaeyoungjo99/particle_simulation.git
```

# Build the package
```bash
$ cd ~/catkin_ws
$ catkin_make
```

# Run the package
```bash
$ source devel/setup.bash
$ roslaunch particle_simulation particle_simulation.launch
```
