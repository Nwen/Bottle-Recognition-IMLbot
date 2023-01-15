# SIMPLE IMAGE RECOGNITION ROBOT IN ROS USING A YOLOv5 SELF-TRAINED MODEL

2023 IML Project

Gwendal Paton - g8paton@enib.fr

Requires ROS2 and Python.

I used ROS2 Humble & Python 3.10.6

## INSTALLATION

Clone the project

`git clone https://github.com/Nwen/imlbot.git`

I controlled the robot using a joystick and the joy node from ROS.
The joystick I used was an Xbox controller.

`sudo apt install ros2-humble-joy ros2-humble-teleop-tools`

More information : https://index.ros.org/p/joy/
Joystick sonfig tutorial : http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

### 3d models

3d models are provided for the simulation world.
Copy all folders in `imlbot/models` into `home/.gazebo/models` (folder is hidden by default)

### Execution

#### Gazebo simulation

`bash launch.bash`

or

```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch imlbot_gazebo imlbot_launch.py
```

#### Image recognition script

`bash launch_recog.bash`

or

```
source /opt/ros/humble/setup.bash
python3 src/imlbot_recognition/scripts/ros_recognition.py
```

## OBJECTIVES & WRITEUP

The goal of this project is to make a simple robot controlled in simulation by an xbox controller. This robot embeds a camera that allows him to perform object recognition. I then trained a model using YOLOv5 to recognise water bottles. The robot is then able to recognise water bottles. This git repo contains the ROS code for the robot behavior and gazebo simulation. 

The creation of the YOLOv5 water bottle recognition model can be found here : <a href="https://colab.research.google.com/drive/1mhDbOoGwjBoeMslo3LR5Rz4uDkE6Re3F?usp=sharing"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"></a>

### High Level description

I used courses about ROS to set up the robot and its environment. I used the lesson about gazebo to set up the simulation. I used knowledge from the computer vision lab to interact with the robot camera and to create the object recognition model using bounding boxes with labels. Lastly Lessons 1 through 5 helped me create, train and validate my image recognition model.

### Challenges 

ROS is a very powerful but complex tool. It takes some time to get used to it and i'm still struggling a lot. I had some trouble with finding the right tutorials for the right version of ROS. Sometimes information is outdated thus making me lose a lot of time trying to fix old things. I also struggled to create a proper simulation world for my robot. I didn't find a lot of water bottles 3d models so I just pasted an image of a water bottle on a cube to try to mimic perpective. I would have liked to have real 3d models. Also the environment is really basic, all the "walls" are white which is not realistic. I would have liked to have an environment closer to real life with a lot of details to ensure that my recognition model worked fine. The last major issue I had was with the training part of my model. In the beginning I was trying to train the model on my own computer but the script wouldn't use my gpu at all making epochs last for at least 1 hour which wasn't going to create me a good model unless I let it train for 100+ hours. I switched to a google collab notebook which didn't have any problem and used a clouded gpu.

### Future work

If I had more time I would use a real robot instead of the one I used which is onlu useful in a simulation environment. I would also add action to my robot, for example it could move towards a water bottle if he sees one.

### Takeaways

• One of the most important part is defining which model/technology to use. I had plenty of choice choosing a technology that could do image recognition but YOLOv5 was in my opinion the better option : it is a powerful deep learning model that you can train yourself. You can tune hyperparameters but all the layers of the models are already there and have proven to be efficient.

• Image recognition requires a lot of computation power to train a model or to use it. When you have a lot of computing power available it is really efficient but otherwise it can cause lag and performance issues. It is hard to make it run smoothly.

## DEMO

https://youtu.be/RyQUWOQm9yE

In the video we can see that the robot can recognise water bottles. Images in teh simulation world are not part from the dataset. The model is not perfect and can make errors on other bottles such as wine bottles. It also seems to detect the human as a water bottle, it might be caused bu the shape of the person, his head looks a bit like a bottle cap and his shirt could be confused with the label. Also cans such as the campbell soup can be confused because they are close to the cylinder shape of a bottle

![image](https://user-images.githubusercontent.com/7102654/212479170-1eacf90d-595a-4f2a-a75b-f10bc133190f.png)

![image](https://user-images.githubusercontent.com/7102654/212479290-e1d322ec-94be-4d1c-96f0-3d7b061d08d3.png)

![image](https://user-images.githubusercontent.com/7102654/212479314-526aaff0-684c-4c2a-8468-0e3b2a6387c1.png)


## SOURCES

Yolov5 by Ultralytics / Glenn Jocher : https://github.com/ultralytics/yolov5

Youtube tutorial (ROS Scripts & robot model) by Robot mania : https://www.youtube.com/watch?v=594Gmkdo-_s
