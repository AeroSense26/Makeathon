The goal behind this project is to make a autonomous, precision application fertilizre robot. It will be able to travel alongside plants and use intelligent sensors to determine the proper dosage, here is the generl idea. All power is already taken care of. Roboflow models are already built. XBGRegession is not built. DO NOT MODIFY THIS README AT ALL IT WILL SERVE AS A CENTRAL DATABASE.

For all code, write it as industry standard and professional as possible. I mean seriously go over the top here.

Hardware:
- An arduino Mega 2560
- A RAMPS 1.4 shield plugged into the arduino mega
- 4x DRV8825 drivers and stepper motors at 200 step/rev, 1.8degree plugged into the x y and z and E0 slots on the Ramps 1.4. These motors are what drive the rover forward or backwards. They are set up like a car but cannot turn, so they need tank like drive almost. The front left wheel is in E0, front right is X, back right is Y, back left is Z. These steppers should move fairly slow
- There are 2x TCRT5000 IR sensors on the front of the rover. They are meant to follow a line of black tape (For example, if the left sensors detects black tape, turn slightly left) The left one is plugged into D16 and the right one is plugged into D17. It should work in conjunction with the move forward and move backward commands, so it just slightly reduces speed in one side until its back on track
- A SEN0546 temp/humidity sensor is also in the SDA/SCL slots 20/21
- A MG996R servo is attached to slot D11. This slot is meant to aim the water cannon from left to right
- A L29N motor driver is attached to D23 and D25. This drives a DC power pump which can deliver the water or suck it back
- The logic is controlled all by rasperry pi 4b connected to the arduino via serial. It can send and receive commands between the arduino, more info below.
- A IMX708 camera is also attached to the Raspberry pi4b for use

Here is the general flow chart of how the rover works.
- The Raspberry Pi receives the command Start in the terminal and starts the cycle.
- It first takes an image, and using a instance segmentation model developed via roboflow (Yolo26) it will try to detect plant stems. (Note, the camera is mounted upsidedown on the rover so flip it in software) If there are no plant stems, it will move one cycle of steps forward (ie. 200 steps). and try again and repeat until it finds a plant. (Note, the camera is facing out towards the left side of the rover)
- Once it finds the plant, it wll take like a weighted average for the average midpoint of the stems. What I mean by this is like the vertical pixels all get collapsed downwrds, and then it finds the average between all those pixels, so if there is more vertical plant to the left the weighed average will be more left. If it is not within 20% of the center of the camera, then move forward or backwards 25 steps and check again. Repeat until its within this threshold.
- Once it is in the midle, find the weighted middle again and move the servo so it is overtop of this point (I know this will take trial and error to find).
- Once it is in the middle, use a seperate Roboflow classification model to classify the plant as either a money_tree, basil, or anthurium. Each of these will have a base level second dispensing of fertilizer.
- Next, it will use a XGBRegression model to determine how much to modify the base fertilzier based on the following input. 1. The time of day (find) (IDk how to input this into XGBRegression), 3. temperature (sense), 4. humidity (sense) 5. VPD (calculate), 6, Average width of the stem for each y pixel row (find using segmentation model). We dont have data yet so we will need to generate synthetic data first time around.
- Next, it will dispense the proper amount of fertilizer sing the pump, before starting it over again. 

Using serial monitor, the Arduino should be able to take in the following commands either the user sends in or the pi sends
- READ_TEMP read the temp sensor and send back the data in the format 00.0, 00.0 for temp and then humidity
- MOVE:FORWARD:0000 Move the stepper motors. Note, 0000 represents the amount of steps, so 0200 is 200 steps. Also, forward can be BACKWARD to go in reverse. It should send back ACK:MOVE:FORWARD:0000 and when its done ACK:MOVE:FORWARD:FINISH of course the related.
- Servo:000 Where 000 represents a number 0-100 that will be found. 0 Is all the way left out of the screen, and 100 is all the way right out of the screen. This will obviously be calibrated via trial and error. Again send ACK:SERVO:000 and ACK:SERVO:FINISH
- PUMP:FORWARD:00 Where 00 represents the time in seconds the pump should run. Note, FORWARD can also be backwards to suck up the water. Again do the ACK:PUMP:FORWARD:00 and ACK:PUMP:FINISH

The user should be able ot enter some commands to the Raspberry Pi via CLI to control the system.
Start will start the entire process described above
Stop will stop it and bring the system slowly to a halt while turning off pump
It should have all the above commands as well

Other info
- The arduino will be the "firmware". It should have the temp/humidit sensor avraging on a constant loop the past 10 data points in the past 10 seconds so when requested, it gives this average instantly. Also, the IR sensor code should be handled entirely arduino side. It should just like reduce speed of correspoding motor no?
- Servo should probably start at 0 position 