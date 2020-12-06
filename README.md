## Project

A ROS package that continuously orientates a robot using a reference.

## Requerements

- docker
- docker-compose

## Install

1. Run the command
```xhost +local:root ```

2. Create a new folder for docker environment.

3. Go to the new folder created.

4. Create a new file called docker-compose.yaml.

```
version: '3'

services:
  turtlesim:
    image: docker.pkg.github.com/ximenesfel/turtlesim_polaris/turtlesim_polaris:1.0.0
    tty: true
    volumes:
      - ./app:/root/catkin_ws/src
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ~/.Xauthority:/root/.Xauthority
    environment:
      DISPLAY: $DISPLAY
      QT_X11_NO_MITSHM: 1
```

5. Clone turtlesim_polaris package.
```git clone https://github.com/ximenesfel/turtlesim_polaris.git```

6. Rename turtlesim_polaris folder to app.
```mv turtlesim_polaris app```

7. Create the container.
```docker-compose up -d```

8. Access the container.
```docker-compose exec turtlesim bash ```

9. Build turtlesim_polaris package
```
cd catkin_ws
catkin_make
```

10. Source the environment
```source ~/.bashrc```

11. Run the launch file
```roslaunch turtlesim_bringup turtlesim.launch```

12. Move Polaris with keyboard arrows keys.

## Result

