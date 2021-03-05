import os
import msvcrt
from dynamixel_sdk import *  
from RT_CPG_units import CPG                  # Uses Dynamixel SDK library
import time 
import matplotlib.pyplot as plt



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

def convert2sComplement(value,bytesize):
    bits = 8*bytesize
    criticalVal = 2**bits

    if value > criticalVal/2:
        value = value - criticalVal

    return value 
    



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
ADDR_CURRENT                = 126

# Protocol version
PROTOCOL_VERSION            = 2.0               #Using protocol 2 

# Default setting
DXL_ID                      = [1,2]               # 1 is hip, 2 is knee 
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'    #should make this an input really 
                                                
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
#check_connection(dxl_comm_result2,dxl_error2)

start = time.time()

hip_command, knee_command, t,currentList,offset = [],[],[],[],[]
cpg = CPG()

while time.time() - start < 20:
    # print("Press any key to continue! (or press ESC to quit!)")
    if msvcrt.kbhit():
        if msvcrt.getch().lower() == b'q':
            break

    #use oscillator to get position
    hip_pos,knee_pos = cpg.get_motor_commands(start,realWorld=True)
    # Write goal position to both motors 
    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_GOAL_POSITION, int(hip_pos))
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_GOAL_POSITION, int(knee_pos))
    
    check_communication(dxl_comm_result1,dxl_error1)
    check_communication(dxl_comm_result2,dxl_error2)


    
    while 1:
        # Read present position
        dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler.read4ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_PRESENT_POSITION)
        dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler.read4ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_PRESENT_POSITION)
        check_communication(dxl_comm_result1,dxl_error1)
        currentRaw,_,_ = packetHandler.read2ByteTxRx(portHandler, DXL_ID[0], ADDR_CURRENT)
        current = convert2sComplement(currentRaw,bytesize=2)
        cpg.torqueFeedback = current
        
        offset.append(cpg.offset)
        currentList.append(current) 
        #print(current)
        hip_command.append(dxl_present_position1)
        knee_command.append(dxl_present_position2)
        t.append(time.time() - start)
        #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[0], dxl_goal_position[index], dxl_present_position1))
#        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[1], dxl_goal_position[index], dxl_present_position2))

        #once error is less than threshold 
        #if abs(hip_pos - dxl_present_position1) < DXL_MOVING_STATUS_THRESHOLD and abs(knee_pos - dxl_present_position2) < DXL_MOVING_STATUS_THRESHOLD:
        break

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

plt.figure()
plt.plot(t,hip_command,"b", label = "Hip")
plt.plot(t,knee_command,"r", label = "Knee")
#plt.plot(t,currentList,'-',label = "Current")
plt.xlabel("Time/s")
plt.ylabel("Motor Position")
plt.title("Actual Motor Positions over a 10 second period")
plt.legend()

plt.figure()
plt.plot(t,currentList)
plt.xlabel("Time/s")
plt.ylabel("Current/2.69mA")

plt.figure()
plt.plot(t,offset)
plt.xlabel("Time/s")
plt.ylabel("Offset")

print(len(t))
plt.show()
