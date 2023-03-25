import time
from pymavlink import mavutil

# Connect to Pixhawk over serial port
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()

# Receive messages from Pixhawk
while True:
    msg = master.recv_msg()
    if msg and msg.get_type() == "STATUSTEXT":
        text = msg.text
        print("Received command:", text)
            # You can add your own logic here to handle the received message
