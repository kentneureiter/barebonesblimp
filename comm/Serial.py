import serial
import struct
import time
import codecs

# Message and data type definitions
MessageType_Parameter = 0x68
DataType_Int = 0x01
DataType_Float = 0x02
DataType_String = 0x03
DataType_Boolean = 0x04

class SerialController:
    def __init__(self, port, baudrate=115200, timeout=2):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            self.serial.reset_input_buffer()
            self.values = None
        except serial.SerialException as se:
            self.serial = None
            self.values = None
            print("Serial port error:", str(se).split(":")[0])
            exit(1)
        
    def manage_peer(self, operation, mac_address=None):
        time.sleep(.02)
        self.serial.reset_input_buffer()
        """Manage ESP-NOW peers by adding or removing them."""
        mac_bytes = bytes(int(x, 16) for x in mac_address.split(':'))
        if operation in ['A', 'R'] and mac_address:
            self.serial.write(operation.encode() + mac_bytes)
            print(f"Peer {'added' if operation == 'A' else 'removed'}: {mac_address}")
        elif operation == 'G':  # Indicates a control command
            self.serial.write(operation.encode() + mac_bytes)
            print("Ground Station Added!")
        else:
            print("Invalid operation or MAC address")
            

        self.wait_for_acknowledgement()

    def send_control_params(self, mac_address, params):
        self.serial.reset_input_buffer()
        if len(params) != 13:
            raise ValueError("Expected 13 control parameters")
        mac_bytes = bytes(int(x, 16) for x in mac_address.split(':'))
        data = struct.pack('<6B13f', *mac_bytes, *params)  # Prefix data with MAC address
        self.serial.write(b'C' + data)  # 'C' indicates a control command
        
        # print(f"Control parameters sent to {mac_address}.")
        self.wait_for_acknowledgement()

    def send_preference(self, peer_mac, value_type, key, value):
        self.serial.reset_input_buffer()
        time.sleep(.02)
        buffer = bytearray()
        if value_type in [DataType_Int, DataType_Float, DataType_Boolean]:
            buffer.extend(struct.pack('<6B', *[int(x, 16) for x in peer_mac.split(':')]))
        buffer.append(MessageType_Parameter)
        buffer.append(value_type)
        buffer.append(len(key))
        buffer.extend(key.encode('utf-8'))
        
        if value_type == DataType_Int:
            buffer.extend(struct.pack('<i', value))
        elif value_type == DataType_Float:
            buffer.extend(struct.pack('<f', value))
        elif value_type == DataType_String:
            buffer.extend(value.encode('utf-8'))
        elif value_type == DataType_Boolean:
            buffer.extend(struct.pack('<?', value))
        
        self.serial.write(b'D' + buffer)
        
        self.wait_for_acknowledgement()
        print("Sending Preference: ", key, ":", value, ", len:", len(buffer))

    def getSensorData(self):
        self.serial.reset_input_buffer()
        self.serial.write(b'I')
        incoming = self.serial.readline()#.decode().strip()
        if (len(incoming) >= 24):
            self.values = struct.unpack('<6f', incoming[0:24])
        else:
            return None
        return self.values

    def wait_for_acknowledgement(self):
        """Wait for an acknowledgment or error message from Arduino."""
        time.sleep(.01)
        try:
            incoming = self.serial.readline()#.decode().strip()
            if incoming == "":
                print("Timeout or no data received.")
                return False
            
            #print("Received from Arduino:", incoming)
            
            self.serial.flush()
            return True
        except serial.SerialException as e:
            print("Serial communication error:", e)
            return False

    def close(self):
        self.serial.close()


# Example usage
if __name__ == "__main__":
    port = 'COM8'  # Adjust as necessary for your setup
    controller = SerialController(port, timeout=.1)  # 5-second timeout

    try:
        # Example: Add a peer
        controller.manage_peer('A', "48:27:E2:E6:E1:00")
        controller.manage_peer('A', 'FF:FF:FF:FF:FF:FF')
        
        # Example: Send control parameters
        mac_address = 'FF:FF:FF:FF:FF:FF'  # Example target peer MAC address
        params = (1.0, 0.5, -1.2, 2.5, 0.0, 1.1, -0.9, 2.2, 3.3, 4.4, 5.5, -2.2, 0.1)
        controller.send_control_params(mac_address, params)
        m1 = 0
        m2 = 0
        s1 = 0
        s2 = 0
        params = (1.0, m1, m2, s1, s2, 0,0,0,0,0,0,0,0)
        controller.send_control_params(mac_address, params)
        controller.send_preference(mac_address, DataType_Int, "controls", 25)
        
    finally:
        controller.close()