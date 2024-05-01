
import dynamixel_sdk as dynamixel


# Control table address
ADDR_PRO_TORQUE_ENABLE       = 64                          # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116
ADDR_PRO_PRESENT_POSITION    = 132
ADDR_PRO_GOAL_VELOCITY       = 104

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 20                             # Dynamixel ID: 20
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0"        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
Opened_Position_value  = 2084                       # Dynamixel will rotate between this value
CLosed_Position_value  = 2600                       # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.PortHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.PacketHandler(2)

index = 1
dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_goal_position = [Opened_Position_value, CLosed_Position_value]    # Goal position
dxl_goal_velocity = 2


dxl_error = 1                                               # Dynamixel error
dxl_present_position = 0                                    # Present position


# Initialize PacketHandler Structs
packet_handler = dynamixel.PacketHandler(PROTOCOL_VERSION)

# Open port
if port_num.openPort():
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    quit()

# Set port baudrate
if port_num.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    quit()


dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_num, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Failed to enable torque, error code: {dxl_comm_result}")
    quit()
elif dxl_error != 0:
    print(f"Failed to enable torque, motor error code: {dxl_error}")
    quit()
else:
    print("Successfully enabled torque!")

dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_num, DXL_ID, ADDR_PRO_GOAL_VELOCITY,dxl_goal_velocity)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Failed to set the goal velocity, error code: {dxl_comm_result}")
    quit()
elif dxl_error != 0:
    print(f"Failed to set the goal velocity, motor error code: {dxl_error}")
    quit()
else:
    print("Successfully set the goal velocity!")

while 1:

    # Write goal position
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_num, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to set the goal position, error code: {dxl_comm_result}")
        quit()
    elif dxl_error != 0:
        print(f"Failed to set the goal position, motor error code: {dxl_error}")
        quit()
    else:
        print("Successfully set the goal position!")

    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_num, DXL_ID,
                                                                                        ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to read present position, error code: {dxl_comm_result}")
            quit()
        elif dxl_error != 0:
            print(f"Failed to read present position, motor error code: {dxl_error}")
            quit()
        else:
            print(f"Present position: {dxl_present_position}")

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))

        if not (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            break

# 等待电机到达目标位置...
# 关闭扭矩
dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_num, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"关闭扭矩失败，错误码: {dxl_comm_result}")
    quit()
elif dxl_error != 0:
    print(f"关闭扭矩失败，电机错误码: {dxl_error}")
    quit()
else:
    print("Succssfully close!")

port_num.closePort()