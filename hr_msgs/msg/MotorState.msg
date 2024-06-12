float64 timestamp    # motor state is at this time
string name          # motor name
float64 position     # current position (in r/s)
float64 speed        # current speed (in r/s)
float64 load         # current load - ratio of applied torque over maximum torque
float64 voltage      # current voltage (V)
int32 temperature    # current temperature (degrees Celsius)
bool torque_enabled  # indicates enabled torque on motor
float64 torque_limit # current limit applied on motor (0-1)
uint8 errorCode      # last error codes

# ErrorCodes:
# for general information see http://support.robotis.com/en/product/actuator/dynamixel/communication/dxl_packet.htm
#
# a value of 0xff(255) means, the motor is in simulation mode
# a value of 0x80(128) means, the motor is offline (no power?)
