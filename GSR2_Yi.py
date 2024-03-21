import rclpy
from rclpy.node import Node
import rclpy.logging as logger
from rclpy.clock import Clock
from sensor_msgs.msg import Imu, MagneticField
from shimmer3_msg_srv.msg import GSR
import serial
import struct

GET_SAMPLING_RATE_COMMAND = 0x03
SAMPLING_RATE_RESPONSE = 0x04
ACK_COMMAND_PROCESSED = 0xFF
STOP_STREAMING_COMMAND = 0x20
START_STREAMING_COMMAND = 0x07
port = '/dev/rfcomm4' # Name of device port

#######################################################################################################
# For every packet that the Shimmer3 receives, it sends an acknowledgement message
# (ACK_COMMAND_PROCESSED) back to the host, to acknowledge receipt of the command.
def wait_for_ack(ser):
    """Wait for acknowledgement bit from device.

    Keyword arguments:
    ser -- serial port
    """
    ddata = ""
    ack = struct.pack('B', ACK_COMMAND_PROCESSED) # pack the given Python values according to the given format. 'B' means unsigned byte, the value passed to struct.pack() should be interpreted as an unsigned byte (8bits)
    # it packs the hexadeicmal value '0xff' into an unsigned btyes.
    while ddata != ack:
        ddata = ser.read(1)
        print(("received [%s]" % struct.unpack('B', ddata)))
        return
  
def accel_calc(data):
    """Calculate m/s^2 values of accelerometer from raw output.

    Keyword arguments:
    data -- raw data from device
    """
    value = (data - 2253) * float(1) / float(920) * float(9.80665)
    # Check Consensys Software -> Configuration for the Values
    return value

def gyro_calc(data):
    """Calculate deg/s values of gyroscope from raw output.

    Keyword arguments:
    data -- raw data from device
    """
    value = data / float(65.5)
    # Check Consensys Software -> Configuration for the Values
    return value

def mag_calc_xy(data):
    """Calculate gauss values of magnometer from raw output, X and Y.

    Keyword arguments:
    data -- raw data from device
    """
    value = data / float(667)
    # Check Consensys Software -> Configuration for the Values
    return value

def mag_calc_z(data):
    """Calculate gauss values of magnometer from raw output, Z.

    Keyword arguments:
    data -- raw data from device
    """
    value = data / float(667)
    # Check Consensys Software -> Configuration for the Values
    return value

def gsr_calc(data, Range):
    """Calculate gsr value.

    Keyword arguments:
    data -- raw data from device
    """
    if (Range == 0):
        p1 = 0.0363
        p2 = -24.8617
    if (Range == 1):
        p1 = 0.0051
        p2 = -3.8357
    if (Range == 2):
        p1 = 0.0015
        p2 = -1.0067
    if (Range == 3):
        p1 = 4.4513e-04
        p2 = -0.3193
    value = (float(p1) * data) + float(p2)
    return value

def ppg_calc(data):
    """PLACEHOLDER."""
    value = data * float(0.00024414062)
    return value

######################################################################################################
class Shimmer(Node):
    def __init__(self, name='GSR2'):
        super().__init__(node_name=name, namespace=name)
        # Log created node
        self.get_logger().info(f'Created Node named {name}')   
        # Calculated Imu publisher
        self.IMUCalcPub = self.create_publisher(Imu, f'Imu/calc', 2)
        # Raw Imu publisher
        self.IMURawPub = self.create_publisher(Imu, f'Imu/raw', 2)
        # EMG publisher            
        self.GSRPub = self.create_publisher(GSR, f'GSR', 2)                     

        ## Calculated Mag publisher (NOT SURE IF WE NEED MAG)
        # self.MAGCalcPub = self.create_publisher(MagneticField, f'Mag/calc', 2)  
        ## Raw Mag publisher (NOT SURE IF WE NEED MAG)
        # self.MAGRawPub = self.create_publisher(MagneticField, f'Mag/raw', 2)    
        ## EXG publisher (NOT SURE WHAT DOES EXG MEAN IN HERE)
        # self.EXGPub = self.create_publisher(EXG, f'EXG', 2)         

    def pubImuCalc(self, imu:Imu) -> bool:
        """
        Summary:
            Function to publish to the IMU Calculated Topic '{name}/Imu/calc'

        Args:
            imu (Imu): Ready to publish Imu message from senor_msgs.msg

        Returns:
            bool: A True if successful and a False if failed
        """
        try:
            self.IMUCalcPub.publish(imu)
            return True
        except:
            self.get_logger().error(f'ERROR, Failed to publish to the {self.get_name}/Imu/calc topic')
            return False
        
    def pubImuRaw(self, imu:Imu) -> bool:
        """
        Summary:
            Function to publish to the IMU Raw Topic '{name}/Imu/raw'

        Args:
            imu (Imu): Ready to publish Imu message from senor_msgs.msg

        Returns:
            bool: A True if successful and a False if failed
        """
        try:
            self.IMURawPub.publish(imu)
            return True
        except:
            self.get_logger().error(f'ERROR, Failed to publish to the {self.get_name}/Imu/raw topic')
            return False    
        
    def pubMagCalc(self, mag:MagneticField) -> bool:
        """
        Summary:
            Function to publish to the MagneticField Calculated Topic '{name}/Mag/calc'

        Args:
            mag (MagneticField): Ready to publish MagneticField message from senor_msgs.msg

        Returns:
            bool: A True if successful and a False if failed
        """
        try:
            self.MAGCalcPub.publish(mag)
            return True
        except:
            self.get_logger().error(f'ERROR, Failed to publish to the {self.get_name}/Mag/calc topic')
            return False
    
    def pubMagRaw(self, mag:MagneticField) -> bool:
        """
        Summary:
            Function to publish to the MagneticField Raw Topic '{name}/Mag/raw'

        Args:
            mag (MagneticField): Ready to publish MagneticField message from senor_msgs.msg

        Returns:
            bool: A True if successful and a False if failed
        """
        try:
            self.MAGRawPub.publish(mag)
            return True
        except:
            self.get_logger().error(f'ERROR, Failed to publish to the {self.get_name}/Mag/raw topic')
            return False

    def pubGSR(self, gsr:GSR) -> bool:
        """
        Summary:
            Function to publish to the GSR Topic '{name}/Emg'

        Args:
            gsr (GSR): Ready to publish GSR message from shimmer3.msg

        Returns:
            bool: A True if successful and a False if failed
        """
        try:
            self.GSRPub.publish(gsr)
            return True
        except:
            self.get_logger().error(f'ERROR, Failed to publish to the {self.get_name}/GSR topic')
            return False
        
###########################################################################################################
def main(args=None):
    rclpy.init(args=args)

    GSR2_node = Shimmer(name='GSR2')
    clock = Clock()

    # IMU message declaration 
    imu = Imu()
    imu.header.frame_id = "imu"
    imu.orientation.x = 0.0
    imu.orientation.y = 0.0
    imu.orientation.z = 0.0
    imu.orientation.w = 0.0
    imu.orientation_covariance = [0.0] * 9
    imu.orientation_covariance[0] = -1.0                # Covariance unknown
    imu.angular_velocity_covariance = [0.0] * 9
    imu.angular_velocity_covariance[0] = -1.0           # Covariance unknown
    imu.linear_acceleration_covariance = [0.0] * 9
    imu.linear_acceleration_covariance[0] = -1.0        # Covariance unknown

    # Raw IMU message declaration 
    imu_raw = Imu()
    imu_raw.header.frame_id = "imu"
    imu_raw.orientation.x = 0.0
    imu_raw.orientation.y = 0.0
    imu_raw.orientation.z = 0.0
    imu_raw.orientation.w = 0.0
    imu_raw.orientation_covariance = [0.0] * 9
    imu_raw.orientation_covariance[0] = -1.0            # Covariance unknown
    imu_raw.angular_velocity_covariance = [0.0] * 9
    imu_raw.angular_velocity_covariance[0] = -1.0       # Covariance unknown
    imu_raw.linear_acceleration_covariance = [0.0] * 9
    imu_raw.linear_acceleration_covariance[0] = -1.0    # Covariance unknown

    ## Magnetometer message declaration 
    # mag = MagneticField()
    # mag.header.frame_id = "imu"
    # mag.magnetic_field_covariance = [0.0] * 9
    # mag.magnetic_field_covariance[0] = -1.0             # Covariance unknown

    ## Raw Magnetometer message declaration
    # mag_raw = MagneticField()
    # mag_raw.header.frame_id = "imu"
    # mag_raw.magnetic_field_covariance = [0.0] * 9
    # mag_raw.magnetic_field_covariance[0] = -1.0         # Covariance unknown

    # GSR message declaration
    gsr = GSR()
    gsr.header.frame_id = "imu"
    gsr.gsr = 0.0
    gsr.gsrrange = 0.0
    gsr.adc12 = 0.0
    gsr.adc13 = 0.0

    # Open Serial Connections with GSR Shimmer
    ser = serial.Serial(port, 115200)
    GSR2_node.get_logger().info(f'Serial Object Created {ser}')
    ser.flush()
    
    # Stop Streaming
    ser.write(struct.pack('B', STOP_STREAMING_COMMAND))
    wait_for_ack(ser)
    
    # Get Sampling Rate
    ser.write(struct.pack('B', GET_SAMPLING_RATE_COMMAND))
    wait_for_ack(ser)
    ddata = ser.read(3)
    ddata = struct.unpack('<BH', ddata)
    print('Sampling Rate is: %f' % (32768 / ddata[1]))

    # Start Streaming
    ser.write(struct.pack('B', 0x07))
    wait_for_ack(ser)

    numbytes = 0
    ddata = b''
    framesize = 1 + 3 + 2*3 + 2*3 + 2*3 + 2 + 2

    while rclpy.ok():
        try:
            while numbytes < framesize:
                ddata += ser.read(framesize)
                numbytes = len(ddata)

            data = ddata[0:framesize]
            numbytes = 0
            ddata = b''

            (packettype) = struct.unpack('B', data[0:1]) #[0]

            ## Linear Acceleration
            (lin_acc_raw_x, lin_acc_raw_y, lin_acc_raw_z) = struct.unpack('HHH', data[4:10])
            lin_acc_cal_x = accel_calc(lin_acc_raw_x)
            lin_acc_cal_y = accel_calc(lin_acc_raw_y)
            lin_acc_cal_z = accel_calc(lin_acc_raw_z)

            ## Extract PPG and GSR
            (PPG_raw, GSR_raw) = struct.unpack('HH', data[10:14])

            ## Calculate PPG
            ppg_cal = ppg_calc(PPG_raw)

            ## Calculate GSR
            # GSR range
            Range = ((GSR_raw >> 14) & 0xff)
            gsr_cal = gsr_calc(GSR_raw, Range)

            ## Angular Velocity
            (ang_vel_raw_x, ang_vel_raw_y, ang_vel_raw_z) = struct.unpack('>hhh', data[14:20])
            ang_vel_cal_x = gyro_calc(ang_vel_raw_x)
            ang_vel_cal_y = gyro_calc(ang_vel_raw_y)
            ang_vel_cal_z = gyro_calc(ang_vel_raw_z)

            ## Magnetic Field
            # (mag_fld_raw_x, mag_fld_raw_z, mag_fld_raw_y) = struct.unpack('>hhh', data[20:26])
            # mag_fld_cal_x = mag_calc_xy(mag_fld_raw_x)
            # mag_fld_cal_y = mag_calc_xy(mag_fld_raw_y)
            # mag_fld_cal_z = mag_calc_z(mag_fld_raw_z)

            # Fill in CALCULATED IMU Message
            imu.linear_acceleration.x = float(lin_acc_cal_x)
            imu.linear_acceleration.y = float(lin_acc_cal_y)
            imu.linear_acceleration.z = float(lin_acc_cal_z)
            imu.angular_velocity.x = float(ang_vel_cal_x)
            imu.angular_velocity.y = float(ang_vel_cal_y)
            imu.angular_velocity.z = float(ang_vel_cal_z)
            # Fill in RAW IMU Message
            imu_raw.linear_acceleration.x = float(lin_acc_raw_x)
            imu_raw.linear_acceleration.y = float(lin_acc_raw_y)
            imu_raw.linear_acceleration.z = float(lin_acc_raw_z)
            imu_raw.angular_velocity.x = float(ang_vel_raw_x)
            imu_raw.angular_velocity.y = float(ang_vel_raw_y)
            imu_raw.angular_velocity.z = float(ang_vel_raw_z)
            # Fill in GSR Message
            gsr.gsr = float(gsr_cal)
            gsr.gsrrange = float(Range)
            gsr.adc12 = float(0.0)
            gsr.adc13 = float(ppg_cal)

        except serial.SerialException as e:
            GSR2_node.get_logger().error(f'SerialException while streaming GSR data. Error is:\n{e.strerror}')

        except KeyboardInterrupt as key:
            GSR2_node.get_logger().info(f"Control C called: {key}")
        
        # Get current ROS Time and fill the stamps of each message
        time = clock.now().to_msg()
        imu_raw.header.stamp = time
        imu.header.stamp = time
        # mag_raw.header.stamp = time
        # mag.header.stamp = time
        gsr.header.stamp = time

        # Publish Messages
        GSR2_node.pubImuCalc(imu)
        GSR2_node.pubImuRaw(imu_raw)
        # GSR2_node.pubMagCalc(mag)
        # GSR2_node.pubMagRaw(mag_raw)
        GSR2_node.pubGSR(gsr)

        # Log that publishing completed
        GSR2_node.get_logger().debug(f'Finished Publishing data to topics')

    # ==== End of While loop that continues until ROS has been killed ==== #
    # ==================================================================== #

    # Send stop streaming command
    ser.write(struct.pack('B', 0x20))
    wait_for_ack(ser)

    # Close the socket
    ser.close()

    # Destroy GSR node
    GSR2_node.destroy_node()
    
    # Shutdown node
    rclpy.shutdown()
    print(f'Streaming has been stopped, end of {__name__} script')

# ==== End of Main() ==== #
# ======================= #
    
if __name__ == '__main__':
    main()       




    

