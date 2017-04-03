# machine-learning
# machine-learning
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

