we are working on a very complex project. It will be a rover for precision fertilizer applicaton. here is the hardware and setup

The rover uses a arduino mega as firmware and it talks to a raspberry pi 4b 2gb. The raspberry pi will send commands through serial to the arduino that will tell it what to do

The rover will start at a position, when activated, the pi tells the arduino to move 4 stepper motors a few inches forward. The pi will take a picture using an IMX708 camera. Using roboflow, it will try to detect if there is a plant in frame or not. if there isnt, move forward, else stay stationary.

Also, we have 2 IR sensors on either side of a line of black tape to help ensure that the rover doesnt drift. Drifting is handled via arduino side

Once a plant is in frame, the arduino will move a servo to position a tube in the frame of the plant

Next, the pi will classify the plant using roboflow, it will use a xgbregression model to determine the proper dosage of fertilizer. It will then control a pump via the arduino to fertilize the robot

This is the basic idea of the project