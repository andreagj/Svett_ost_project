# ros_pkg_template
Template for an Ascend ROS package

## Getting started:
1. Clone this repo into your catkin_ws/src directory
2. Clone [ascend_msgs](https://github.com/AscendNTNU/ascend_msgs) into your catkin_ws/src directory, if not done already.
2. Change project/package name in package.xml, CMakeLists.txt, Dockerfile and Doxyfile
3. Rename folder and remove .git directory
4. Run git init and add your github repository as remote.
5. Add your own source and header files in src and include folder
6. Add build and run dependencies if neccesary
7. Add your nodes to CMakeLists.txt
8. Write a descriptive README.md
9. Run catkin_make in your catkin_ws directory and make sure it builds.
10. Commit and push your changes to Github!
11. Start developing!

## Sublime project file
The package template also contains a sublime project file which contains the following:
- Build system: catkin_make
- Build system: docker build
- Indentation settings: 4 spaces 

Project files for other IDEs might be added in the future!
