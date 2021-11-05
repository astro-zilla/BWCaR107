# BWCaR107
IDP software for team 107
## Project Structure:
- Daedalus is software to run on a PC, which reads camera data and sends commands to the Arduino.
  - `daedalus.aruco` contains a selection of wrapped functions from `cv2.aruco` which are used to identify the ArUco marker on top of the robot.
  - `daedalus.cam_skeleton` is a skeleton file for programs using the cv2 camera library
  - `daedalus.Image` contains functions for image transformations and undistortion 
  - `daedalus.navigation` contains functions for finding objects n the frame.
  - `daedalus.peripherals` contains a functions used to read and wrte from Arduino peripherals
  - `daedalus.square_picker` is a utility file used to select a quadrilateral used to form an affine transform, squaring an image on the quadrilateral.
  - `daedalus.streaming` includes 2 classes, `ArduinoStream` and `VideoStream`, both of which inherit `threading.Thread` and are run in their own logical threads.
- Theseus is software to run on an Arduino, which translated commands from Daedalus into realisable actions of motors, servos and LEDs. It also relays sensor and state information back to Daedalus for analysis.
## To Use:
- `Daedalus\main.py` will run without the required connections but with no functionality.
- `Theseus\wifi-secrets.h` requires token information which you can generate on the UCS tokens page.
- `Theseus\theseus.ino` requires you to enter the local IP of the computer you are running Daedalus on as `IPAddress server`, which must be connected to eduroam.
- `Daedalus\main.py` must run in a terminal or terminal emulator, not a python console.
- You must ssh tunnel into `gate.eng.cam.ac.uk` to get access to the cameras:
  - ~ ssh -L 8081:idpcam1.eng.cam.ac.uk:8080 USER@gate.eng.cam.ac.uk
  - ~ ssh -L 8082:idpcam2.eng.cam.ac.uk:8080 USER@gate.eng.cam.ac.uk