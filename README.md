# BWCaR107
IDP project for team 107
## Project Structure
- Daedalus is software to run on a PC, which reads camera data and sends commands to the Arduino.
  - `daedalus.streaming` includes 2 classes, `ArduinoStream` and `VideoStream`, both of which inherit `threading.Thread` and are run in their own logical threads.
  - `daedalus.aruco` contains a selection of wrapped functions from `cv2.aruco` which are used to identify the ArUco marker on top of the robot.
  - `daedalus.navigation` contains functions for 
