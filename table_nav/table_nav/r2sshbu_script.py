import os

# Open a new terminal window and execute the SSH command
os.system('gnome-terminal --tab -- sshrp user@remote_device')

# Wait for the SSH connection to be established
connected = False
while not connected:
    response = os.system('ping -c 1 remote_device')
    if response == 0:
        connected = True

# Execute the ROS bringup command
os.system('rosbu')

# Check if the ROS bringup command was executed successfully
rosbu_success = False
while not rosbu_success:
    response = os.system('rosnode list')
    if "turtlebot3_bringup" in response:
        rosbu_success = True

print("Connected to remote device and ROS bringup completed successfully.")