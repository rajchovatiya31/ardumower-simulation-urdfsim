# Simulation of Ardumower in Urdfsim
[**Ardumower**](http://www.ardumower.de) is a DIY lawnmower built on Arduino. Currently, it uses a perimeter sensor to localize itself and stay within the boundary. This project is an initial step toward replacing the perimeter sensor with the computer vision to detect grass and follow the boundary of grass. The idea is to deploy PID wall following algorithm in pixel unit instead of meter. The first boundary is extracted from the segmentation mask. For finding the distance to the boundary in pixels, I made virtual 2d lidar using a simple ray casting algorithm with OpenCV.

## Getting Started
Here, [**Urdfsim**](https://github.com/mitchellspryn/UrdfSim) is used for simulation. It is a fork of the AirSim simulator that attempts to solve the problem of simulation of arbitrary robots. The custom environment must be made. 

### Prerequisites
1. Microsoft Visual Studio (if using windows)
2. Unreal Engine
3. Urdfsim
4. Opencv
5. Scikit-image

### Test Environment
- Windows: 8.1
- Visual Studio 2017: 15.8
- Unreal Engine: 4.18
- Python: 3.7
 

### Installing

1. Install and setup [**Visual Studio**](https://visualstudio.microsoft.com/de/downloads/) (**for windows**)
    > **_Note:_** Make sure your visual studio version matches with version mentioned in requirement of Urdfsim and also to install VC++ and Windows SDK 8.x
    
    
2.  Build [**Urdfsim**](https://github.com/mitchellspryn/UrdfSim)
3.  Setup environment for Urdfsim
    > **_Note:_** Building and setting up an environment for Urdfsim is the same as Airsim. Instructions can be found here [:link:](https://microsoft.github.io/AirSim/)
    
4. Install OpenCV and Scikit-image

### Setup
1. Set up your custom environment for Urdfsim. The instructions can be found here [:link:](https://microsoft.github.io/AirSim/docs/unreal_custenv/)
    > **_Note:_** Some prebuilt environments can be found here[:link:](https://github.com/Microsoft/AirSim/releases)

2. Copy settings.json file to:
    - For Windows: `Documents\AirSim`
    - For Linux: `~/Documents/AirSim`

3. Change **UrdfFile** tag to the absolute path of XML file of robot description or `path to/where/cloned/this/repo/ardumower.xml`

    > **_Note:_**
    > - More instructions for setting file: [:link:](https://github.com/microsoft/AirSim/blob/master/docs/settings.md)
    > - Additional settings for Urdfsim: [:link:](https://github.com/mitchellspryn/UrdfSim/blob/master/docs/UrdfSettings.md)

### Runnig Simulation
1. Open visual studio project file(.sln) generated during 'setup of unreal environment' stage.

2. Select your Unreal project as Start Up project and make sure Build config is set to "Develop Editor" and x64 and start to debug the project.

3. After Unreal Editor loads press the Play button.
4. Wait until the model loads into the environment. Then run `python/simulation.py`.

> **_Note:_** Make sure you have set the `GameMode Override` to `AirSimGameMode` in Window/World Settings before pressing play button.



### Discussion
Here, I use the only feature of Airsim to get segmentation images of the environment. For real world, the user may have their own pre-trained neural net to predict the grass.

### Results
Some videos that illustrate my idea!!

![](./output/output.gif)

### Acknowledgments
- Reactive wall following robot with laser scanner sensor (https://github.com/ssscassio/ros-wall-follower-2-wheeled-robot)

