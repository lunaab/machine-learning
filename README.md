# machine-learning

Data collection steps:

0) Run roscore in a terminal
1) Plug the ASUS camera into a USB port
   a) Launch ros to connect camera by executing this command:
        roslaunch openni2_launch openni2.launch
      in a terminal.
2) Plug the USB to B Cable into the Arduino
   a) Open the ardunio IDE
   b) open the "AccelGyro" sketch
   c) GOTO: Tools/Board menu and select "Arduino Duelmilanova"
   d) GOTO: Tools/Port menu and select the available USB port number
   e) upload the sketch to the board
3) Navigate to the asus_image_depth.py file
   a) Run by "python asus_image_depth.py <filename w/o extension>"
   b) End by "^C" and wait as the data is saved

File Descriptions:

AccelGyro.ino
    Used to collect data from the accelerometer

NavSys
    Directory containing the necessary files to run the
    navigation system

data_read_utils.py
    Utility python file containing useful methods for
    reading image and depth data for this project.
    Also contains methods for converting image data from
    floats to uint8s and to view collected data
 
depth_network_prepro.py
    Depth network using preprocessing on data. Does not
    perform nearly as well. Depreciated.

depth_predictor_node.py
    Rospy node for publishing depth predictions

asus_image_depth.py
    Used to collect image and depth data

depth_network.py
    Network architecture is defined here. OO approach
    Training is done by "python depth_network.py"

network_utils.py
    Used for functions that are used across multiple
    instances of DepthNetwork. Contains loss function
    mse_ignore_nan    

pca.py
    old

RunMotorsThroughSerialInput
    directory containing ino file to run the vibration
    motors through serial input
