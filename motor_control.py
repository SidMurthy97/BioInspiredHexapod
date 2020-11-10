import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def check_connection(dxl_comm_result,dxl_error):
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

def check_communication(dxl_comm_result,dxl_error):

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))




if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Initialise addresses based on control table data from data sheel 
ADDR_PRO_TORQUE_ENABLE      = 64               #must be enabled to allow for motor to turn
ADDR_PRO_GOAL_POSITION      = 116              #register to write goal position to 
ADDR_PRO_PRESENT_POSITION   = 132              #register to read goal position from 

# Protocol version
PROTOCOL_VERSION            = 2.0               #Using protocol 2 

# Default setting
DXL_ID                      = [1,2]               # 1 refers to standalone motor, 2 to the motor attached to leg - might want to change this to [0,1] to keep in line with indices
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM5'    #should make this an input really 
                                                
#define some constants                                                 
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 2500         # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                #Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque on both motors
dxl_comm_result1, dxl_error1 = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result2, dxl_error2 = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
check_connection(dxl_comm_result1,dxl_error1)
check_connection(dxl_comm_result2,dxl_error2)

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Write goal position to both motors 
    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
    
    check_communication(dxl_comm_result1,dxl_error1)
    check_communication(dxl_comm_result2,dxl_error2)

    while 1:
        # Read present position
        dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler.read4ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_PRESENT_POSITION)
        dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler.read4ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_PRESENT_POSITION)
        check_communication(dxl_comm_result1,dxl_error1)
        
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[0], dxl_goal_position[index], dxl_present_position1))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[1], dxl_goal_position[index], dxl_present_position2))

        #once error is less than threshold 
        if abs(dxl_goal_position[index] - dxl_present_position1) < DXL_MOVING_STATUS_THRESHOLD and abs(dxl_goal_position[index] - dxl_present_position2) < DXL_MOVING_STATUS_THRESHOLD:
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0


# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
# Close port
portHandler.closePort()
