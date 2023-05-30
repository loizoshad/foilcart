# Run

In order to upload either a unit test or the main file on the Teensy 4.1 run the following command, where <env> denotes the name of the environment (You can find the list of environments in the platformio.ini file)

- $ pio run --environment <env> --target upload

# TODO

- Measurement Model:
    - Create a nonlinear model based on how you want to handle the Senix sonar sensor measurements.
    - In order to define that nonlinear model, you can follow the same reasoning as with the definition of the nonlinear dynamics model in the file (dynamic_model.py)

- CAN bus:
    - Ask Nick to fix the transmission of CAN signals from the control input and Senix sensor nodes.
    - After that is fixed, you need to correctly parse the received signals from these two nodes.

- EKF:
    - When the measurmenet model and CAN bus issues mentioned above are fixed, then you need to include the signals in the EKF loop.


- Dynamics Model:
    - There is a possibility that the definition of the definition of the torque induced by the gravity force is not correct in the model.