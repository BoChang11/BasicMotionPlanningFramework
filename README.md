# Basic Motion Planning FrameWork
Include RRT planner, and state of the art GUST planner - Erion Plaku (Region-Guided and Sampling-Based Tree Search )

## Prerequisites
- OPENGL library
- GLUT library
- BOOST library
- Pybullet library


## Getting Started
```
cmake -DCMAKE_BUILD_TYPE="Release"
make
./bin/Runner GRunPlanner "sceneName" "queryName" "plannerType"
```

Example:
```
./bin/Runner GRunPlanner sceneA.txt query0.txt 1
```
plannerType default = 1
0: RRT planner
1: GUST planner
- Press "r" until solved.

## License

Please contact me via lesun90@gmail.com if you want to use this code for your work.
