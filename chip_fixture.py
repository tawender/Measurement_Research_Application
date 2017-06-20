import smbus
bus = smbus.SMBus(1)


class ChipFixture(object):
    def __init__(self,i2c_address,bus=1):
        self.bus = bus
        if ((i2c_address >= 1) and (i2c_address <= 0x7f)):
            self.i2c_addr = i2c_address
        else:
            raise RuntimeError("Invalid I2C address (%#0x) creating Fixture object"%(i2c_address))
        self.num_sensor_bytes = 4

    def readSensor(self,sensor_num):
        """read an individual sensor number and return resistance
           in ohms
        """
        if ((sensor_num < 1) or (sensor_num > 44)):
            sensor_num = 1 #default if out of range

        list_of_bytes = bus.read_i2c_block_data(self.i2c_addr,0x50+sensor_num,4)

        return self.bytesToFloat(list_of_bytes)

    def bytesToFloat(self, l):
        """converts a list of 4 bytes to a 32-bit floating point value
        """
        return struct.unpack('!f', self.bytesToHexString(l).decode('hex'))[0]

    def bytesToHexString(self,l):
        """converts a list of 4 bytes (LSB first) to a string of hex characters
           without '0x' at the beginning
        """
        return hex( l[0] + (l[1]<<8) + (l[2]<<16) + (l[3]<<24) )[2:]

    def readSensors(self):
        """reads all gas concentration sensors on a chip fixture and reads
           the temperature, humidity, and pressure from the BME280 sensor
        """
        NUM_SENSORS = 25    #read sensors 1-22,T,H,P = 25
        list_of_bytes = bus.read_i2c_block_data(self.i2c_addr,0x50,NUM_SENSORS*self.num_sensor_bytes)

        float_results = []
        next_sensor_list = []
        byteCounter = 1;

        for byte in results:

            next_sensor_list.append(byte)
            if byteCounter in [1,2,3]:
                byteCounter+=1
            elif byteCounter==4:
                float_results.append(self.bytesToFloat(next_sensor_list))
                byteCounter=1
                next_sensor_list = []

        return float_results