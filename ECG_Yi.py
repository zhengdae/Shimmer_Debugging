import rclpy
from rclpy.node import Node
import rclpy.logging as logger
from rclpy.clock import Clock
from sensor_msgs.msg import Imu, MagneticField
from shimmer3_msg_srv.msg import EMG, EXG
import serial
import struct

GET_SAMPLING_RATE_COMMAND = 0x03
SAMPLING_RATE_RESPONSE = 0x04
ACK_COMMAND_PROCESSED = 0xFF
STOP_STREAMING_COMMAND = 0x20
START_STREAMING_COMMAND = 0x07

port = '/dev/rfcomm1' # Name of device port
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
    
def emg_calc(data):
    """
    Summary:
        Returns the calculated EMG value (data* 0.00002404054)

    Args:
        data (float): Raw EMG channel value

    Returns:
        value (float): Adjusted EMG value
    """
    value = float(data) * float(0.00002404054) 
    # Check EMG User Manual for the Calculation of 0.00002404054
    return value
  
def accel_calc(data):
    """Calculate m/s^2 values of accelerometer from raw output.

    Keyword arguments:
    data -- raw data from device
    """
    value = (data - 2047) * float(1) / float(830) * float(9.80665)
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
    value = data / float(670)
    # Check Consensys Software -> Configuration for the Values
    return value

def mag_calc_z(data):
    """Calculate gauss values of magnometer from raw output, Z.

    Keyword arguments:
    data -- raw data from device
    """
    value = data / float(600)
    # Check Consensys Software -> Configuration for the Values
    return value

######################################################################################################
class Shimmer(Node):
    def __init__(self, name='ECG'):
        super().__init__(node_name=name, namespace=name)
        # Log created node
        self.get_logger().info(f'Created Node named {name}')   
        # Calculated Imu publisher
        self.IMUCalcPub = self.create_publisher(Imu, f'Imu/calc', 2)
        # Raw Imu publisher
        self.IMURawPub = self.create_publisher(Imu, f'Imu/raw', 2)
        # EMG publisher            
        self.EMGPub = self.create_publisher(EMG, f'EMG', 2)                     

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

    def pubEXG(self, exg:EXG) -> bool:
        """
        Summary:
            Function to publish to the EXG Topic '{name}/Exg'

        Args:
            exg (EXG): Ready to publish EXG message from shimmer3.msg

        Returns:
            bool: A True if successful and a False if failed
        """
        try:
            self.EXGPub.publish(exg)
            return True
        except:
            self.get_logger().error(f'ERROR, Failed to publish to the {self.get_name}/EXG topic')
            return False

    def pubEMG(self, emg:EMG) -> bool:
        """
        Summary:
            Function to publish to the EMG Topic '{name}/Emg'

        Args:
            emg (EMG): Ready to publish EMG message from shimmer3.msg

        Returns:
            bool: A True if successful and a False if failed
        """
        try:
            self.EMGPub.publish(emg)
            return True
        except:
            self.get_logger().error(f'ERROR, Failed to publish to the {self.get_name}/EMG topic')
            return False
        
###########################################################################################################
def main(args=None):
    rclpy.init(args=args)

    ECG = Shimmer(name='ECG')
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

    ## EXG message declaration
    ## exg = EXG()
    # exg.header.frame_id = "imu"
    # exg.c1status = 0.0
    # exg.c1ch1 = 0.0
    # exg.c1ch2 = 0.0
    # exg.c2status = 0.0
    # exg.c2ch1 = 0.0
    # exg.c2ch2 = 0.0

    # EMG message declaration
    emg = EMG()
    emg.header.frame_id = "imu"
    emg.emg_c1ch1 = 0.0
    emg.emg_c1ch2 = 0.0

    # Open Serial Connections with ECG Shimmer
    ser = serial.Serial(port, 115200)
    ECG.get_logger().info(f'Serial Object Created {ser}')
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
    framesize = 1 + 3 + 2*3 + 2*3 + 2*3 + 7
    while rclpy.ok():
        try:
            while numbytes < framesize:
                ddata += ser.read(framesize)
                numbytes = len(ddata)

            data = ddata[0:framesize]
            numbytes = 0
            ddata = b''

            # Get RAW Linear acceleration data
            (lin_acc_raw_x, lin_acc_raw_y, lin_acc_raw_z) = struct.unpack('HHH', data[4:10])
            # Get CALCULATED Linear acceleration data
            lin_acc_cal_x = accel_calc(lin_acc_raw_x)
            lin_acc_cal_y = accel_calc(lin_acc_raw_y)
            lin_acc_cal_z = accel_calc(lin_acc_raw_z)

            # Get RAW Angular velocity data
            (ang_vel_raw_x, ang_vel_raw_y, ang_vel_raw_z) = struct.unpack('>hhh', data[10:16])
            # Get CALCULATED Angular velocity data
            ang_vel_cal_x = gyro_calc(ang_vel_raw_x)
            ang_vel_cal_y = gyro_calc(ang_vel_raw_y)
            ang_vel_cal_z = gyro_calc(ang_vel_raw_z)

            ##  Get RAW magnetic field data
            # (mag_fld_raw_x, mag_fld_raw_z, mag_fld_raw_y) = struct.unpack('>hhh', data[16:22])
            ## Get CALCULATED magnetic field data
            # mag_fld_cal_x = mag_calc_xy(mag_fld_raw_x)
            # mag_fld_cal_y = mag_calc_xy(mag_fld_raw_y)
            # mag_fld_cal_z = mag_calc_z(mag_fld_raw_z)

            # Get EMG values (Channel 1 and Channel 2)
            emg_c1ch1_raw = struct.unpack('>i', (data[23:26] + b'\0'))[0] >> 8
            emg_c1ch2_raw = struct.unpack('>i', (data[26:29] + b'\0'))[0] >> 8
            # Get Calculated EMG values (Channel 1 and Channel 2)
            emg_c1ch1_cal = emg_calc(emg_c1ch1_raw)
            emg_c1ch2_cal = emg_calc(emg_c1ch2_raw)  

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

            ## Fill in CACULATED MAG Message
            # mag.magnetic_field.x = mag_fld_cal_x
            # mag.magnetic_field.y = mag_fld_cal_y
            # mag.magnetic_field.z = mag_fld_cal_z
                        
            ## Fill in RAW MAG Message
            # mag_raw.magnetic_field.x = mag_fld_raw_x
            # mag_raw.magnetic_field.y = mag_fld_raw_y
            # mag_raw.magnetic_field.z = mag_fld_raw_z

            # Fill in EMG Message
            emg.emg_c1ch1 = emg_calc(emg_c1ch1_cal)
            emg.emg_c1ch2 = emg_calc(emg_c1ch2_cal)

        except serial.SerialException as e:
            ECG.get_logger().error(f'SerialException while streaming ECG data. Error is:\n{e.strerror}')

        except KeyboardInterrupt as key:
            ECG.get_logger().info(f"Control C called: {key}")
        
        # Get current ROS Time and fill the stamps of each message
        time = clock.now().to_msg()
        imu_raw.header.stamp = time
        imu.header.stamp = time
        # mag_raw.header.stamp = time
        # mag.header.stamp = time
        emg.header.stamp = time

        # Publish Messages
        ECG.pubImuCalc(imu)
        ECG.pubImuRaw(imu_raw)
        # ECG.pubMagCalc(mag)
        # ECG.pubMagRaw(mag_raw)
        ECG.pubEMG(emg)

        # Log that publishing completed
        ECG.get_logger().debug(f'Finished Publishing data to topics')

    # ==== End of While loop that continues until ROS has been killed ==== #
    # ==================================================================== #

    # Send stop streaming command
    ser.write(struct.pack('B', 0x20))
    wait_for_ack(ser)

    # Close the socket
    ser.close()

    # Destroy ECG node
    ECG.destroy_node()
    
    # Shutdown node
    rclpy.shutdown()
    print(f'Streaming has been stopped, end of {__name__} script')

# ==== End of Main() ==== #
# ======================= #
    
if __name__ == '__main__':
    main()       




    

