
import serial
import time
import numpy as np
import requests


class InspireHand:
    """
    A class to control a Inspire hand device via serial communication.
    """
    
    # Register address dictionary
    # Serial Manual at https://cdn.shopify.com/s/files/1/0634/6863/4356/files/INSPIRE-ROBOTS-THE-DEXTEROUS-HAND-RH56-SERIES-USER-MANUAL.pdf?v=1728897833
    
    # Address for each register
    REGISTERS = {
        'ID': 1000, 
        'baudrate': 1002, 
        'clearErr': 1004,
        'forceClb': 1009,
        'angleSet': 1486,
        'forceSet': 1498,
        'speedSet': 1522,
        'angleAct': 1546,
        'forceAct': 1582,
        'errCode': 1606,
        'statusCode': 1612,
        'temp': 1618,
        'actionSeq': 2320,
        'actionRun': 2322
    }
    # BAUDRATES = {
    #     0: 115200,
    #     1: 57600,
    #     2: 19200
    # }
    
    def __init__(self, port, baudrate=0, hand_id=1):
        """
        Initialize the RoboticHand with serial connection.
        
        Args:
            port (str): Serial port to connect to (e.g. '/dev/ttyUSB0')
            baudrate_id (int): Baudrate ID (0: 115200, 1: 57600, 2: 19200)
            hand_id (int): ID of the hand device
        """

        self.port = port # 
        self.hand_id = hand_id # default 1, range 1-254, each hand in same bus should have different ID
        self.baudrate = baudrate # 115200

        self.ser = None
        self.finger_angle_buf = []
        self.buffer_len = 4
        self.finger_cmd = np.array([-1] * 6)
        # self.connect()
        # self.initialize()
        

        
    def connect(self):
        """
        Open serial connection to the hand device.
        
        Returns:
            bool: True if connection was successful, False otherwise
        """
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baudrate
            self.ser.open()
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Error connecting to hand: {e}")
            return False
            
    def disconnect(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from hand")
    
    def write_register(self, add, num, val):
        """
        Write values to a register.
        
        Args:
            add (int): Register address
            num (int): Number of bytes to write
            val (list): Values to write
        """
        if not self.ser or not self.ser.is_open:
            print("Serial connection not open. check {} is 777".format(self.port))
            return
            
        bytes_to_write = [0xEB, 0x90]
        bytes_to_write.append(self.hand_id)  # id
        bytes_to_write.append(num + 3)  # len
        bytes_to_write.append(0x12)  # cmd
        bytes_to_write.append(add & 0xFF)
        bytes_to_write.append((add >> 8) & 0xFF)  # add
        
        for i in range(num):
            bytes_to_write.append(val[i])
            
        checksum = 0x00
        for i in range(2, len(bytes_to_write)):
            checksum += bytes_to_write[i]
        checksum &= 0xFF
        bytes_to_write.append(checksum)
        
        self.ser.write(bytes_to_write)
        time.sleep(0.01)
        self.ser.flush()
    
    def read_register(self, add, num, mute=True):
        """
        Read values from a register.
        
        Args:
            add (int): Register address
            num (int): Number of bytes to read
            mute (bool): Whether to suppress output
            
        Returns:
            list: Values read from the register
        """
        if not self.ser or not self.ser.is_open:
            print("Serial connection not open")
            return []
            
        bytes_to_write = [0xEB, 0x90]
        bytes_to_write.append(self.hand_id)  # id
        bytes_to_write.append(0x04)  # len
        bytes_to_write.append(0x11)  # cmd
        bytes_to_write.append(add & 0xFF)
        bytes_to_write.append((add >> 8) & 0xFF)  # add
        bytes_to_write.append(num)
        
        checksum = 0x00
        for i in range(2, len(bytes_to_write)):
            checksum += bytes_to_write[i]
        checksum &= 0xFF
        bytes_to_write.append(checksum)
        
        self.ser.write(bytes_to_write)
        time.sleep(0.01)
        recv = self.ser.read_all()
        
        if len(recv) == 0:
            return []
            
        num = (recv[3] & 0xFF) - 3
        val = []
        for i in range(num):
            val.append(recv[7 + i])
            
        if not mute:
            print('Register values read in sequence: ', end='')
            for i in range(num):
                print(val[i], end=' ')
            print()
            
        return val
    
    def write_to_fingers(self, register_name, values):
        """
        Write values to all six fingers.
        
        Args:
            register_name (str): The register name ('angleSet', 'forceSet', or 'speedSet')
            values (list): List of 6 values to write, one for each finger
        """
        if register_name not in ['angleSet', 'forceSet', 'speedSet']:
            print("Invalid register name. Use 'angleSet', 'forceSet', or 'speedSet'")
            return
            
        if len(values) != 6:
            print("Values list must contain exactly 6 elements")
            return
            
        val_reg = []
        for i in range(6):
            val_reg.append(values[i] & 0xFF)
            val_reg.append((values[i] >> 8) & 0xFF)
            
        self.write_register(self.REGISTERS[register_name], 12, val_reg)
    
    def read_from_fingers(self, register_name):
        """
        Read values from all six fingers.
        
        Args:
            register_name (str): The register name to read from
            
        Returns:
            list: Values read from the fingers, or empty list if error
        """
        valid_registers = ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct', 'errCode', 'statusCode', 'temp']
        
        if register_name not in valid_registers:
            print(f"Invalid register name. Use one of: {', '.join(valid_registers)}")
            return []
            
        if register_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct']:
            val = self.read_register(self.REGISTERS[register_name], 12, True)
            if len(val) < 12:
                # print('No data read')
                return []
                
            val_act = []
            for i in range(6):
                val_act.append((val[2*i] & 0xFF) + (val[1 + 2*i] << 8))
                
            
            return val_act
            
        elif register_name in ['errCode', 'statusCode', 'temp']:
            val_act = self.read_register(self.REGISTERS[register_name], 6, True)
            if len(val_act) < 6:
                print('No data read')
                return []
                
            return val_act
    
        
    def set_speed(self, speed):
        """
        Set speed for all fingers.
        
        Args:
            speed (int or list): Speed value (0-1000) for all fingers, or list of 6 values
        """
        if isinstance(speed, (int, float)):
            speed = [int(speed)] * 6
        self.write_to_fingers('speedSet', speed)
    
    def set_force(self, force):
        """
        Set force for all fingers.
        
        Args:
            force (int or list): Force value (0-1000) for all fingers, or list of 6 values
        """
        if isinstance(force, (int, float)):
            force = [int(force)] * 6
        self.write_to_fingers('forceSet', force)
    
    def set_angle(self, angle):
        """
        Set angle for all fingers.
        
        Args:
            angle (int or list): Angle value (0-1000) for all fingers, or list of 6 values
        """
        if isinstance(angle, (int, float)):
            angle = [int(angle)] * 6
        self.write_to_fingers('angleSet', angle)
    
    
    def get_position(self):
        """Get the actual positions of all fingers."""
        return self.read_from_fingers('angleSet')
    
    def get_angle(self):
        """Get the actual angles of all fingers."""
        return self.read_from_fingers('angleAct')
    
    def get_error_code(self):
        """Get error codes for all fingers."""
        return self.read_from_fingers('errCode')
    

    def run_random_test(self, iterations=1000, delay=1.0):
        """
        Run a test with random finger positions.
        
        Args:
            iterations (int): Number of random positions to test
            delay (float): Delay between positions in seconds
        """
        import random
        for i in range(iterations):
            angle = random.randint(0, 1000)
            self.set_angle([angle] * 6)
            time.sleep(delay)
            
    def initialize(self, speed=1000, force=1000):
        """
        Initialize the hand with default settings.
        
        Args:
            speed (int): Initial speed setting
            force (int): Initial force setting
        """
        self.connect()
        self.set_speed(speed)
        self.set_force(force)


# Example usage
if __name__ == '__main__':
    # Create and initialize hand
    left_hand = InspireHand(port='/dev/ttyUSB0')

    left_hand.run_random_test(iterations=100, delay=0.1)
            
