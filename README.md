# CFBSController
C++ implementation of the Command Filtered Backstepping Controller

# Building the project
To build the project, navigate to the directory of the Controller and run the following commands from Command prompt:

`mkdir build`\
`cd build` \
`cmake ..`\
`make`

# Running the project
1. Download the simulink file onto the Target hardware
2. Edit the IP address field in the Send TC/IP and Receive TCP/IP blocks in the Simulink model to the IP address of the hardware on which the Controller will be run
3. Once the project is built on the controller hardware `./controllerCpp` in the console to run the executable. This will initialize a server on the controller and put it into      listening mode
4. Run the Simulink model on the target hardware in paced-mode (real-time). Ensure that controller's server is listening for connections before running the Simulink model. Otherwise, the Simulink's clients will timeout 
5. If connection is established, the simulation will begin
