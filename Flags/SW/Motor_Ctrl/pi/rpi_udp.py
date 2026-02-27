import socket
from motors import *
import time

# ====== CONFIGURATION CONSTANTS =====
MOTOR_START_ANGLE = 0
MOTOR_END_ANGLE = 90
MOTOR_SPEED = 25
MOTOR_MOVE_TIME = 2
PAUSE = 10


# ====== MOTOR CONTROL FUNCTION =====
def raise_flag(motor_number, hold, angle, velocity):
        motor_state.angle = angle * 10
        motor_state.velocity = velocity
        motors_driver.update(motor_number, motor_state)
        time.sleep(hold)

motor_state = MotorState(
    angle=0.0,
    velocity=0.0,
    acceleration=0.0,
    pattern_time=0.0,
    sub_patterns=[],
    color='black',
    width=2,
    active_instrument=None
)
motors_driver = MotorsDriver()

# Define the local IP address and port
UDP_IP = "10.1.149.209"
UDP_PORT = 5005
BUFFER_SIZE = 1024

# ====== UDP SERVER SETUP =====
# Create a datagram socket (UDP)
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind the socket to the address and port
    sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP Server up and listening on {UDP_IP}:{UDP_PORT}")

    while True:
        # Receive data and the sender's address
        data, addr = sock.recvfrom(BUFFER_SIZE)
        if not data:
            break
        print(f"Received message from {addr}: {data.decode('utf-8')}")
        
        data = data.decode('utf-8')
        
        if(data == "spanija"):
            raise_flag(0, MOTOR_MOVE_TIME, MOTOR_END_ANGLE, MOTOR_SPEED)
            time.sleep(PAUSE)
            raise_flag(0, MOTOR_MOVE_TIME, MOTOR_START_ANGLE, MOTOR_SPEED)

        elif(data == "srbija"):
            raise_flag(1, MOTOR_MOVE_TIME, MOTOR_END_ANGLE, MOTOR_SPEED)
            time.sleep(PAUSE)
            raise_flag(1, MOTOR_MOVE_TIME, MOTOR_START_ANGLE, MOTOR_SPEED)
            
        elif(data == "jamajka"):
            raise_flag(2, MOTOR_MOVE_TIME, MOTOR_END_ANGLE, MOTOR_SPEED)
            time.sleep(PAUSE)
            raise_flag(2, MOTOR_MOVE_TIME, MOTOR_START_ANGLE, MOTOR_SPEED)
        else:
            break;
        
        # Send a reply back to the client
        response_message = "Message received by server"
        sock.sendto(response_message.encode('utf-8'), addr)

except socket.error as e:
    print(f"Socket error: {e}")
finally:
    sock.close()
    print("UDP Server closed")

