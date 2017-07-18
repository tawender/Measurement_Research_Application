import struct


class ChipFixture(object):
    NUM_SENSOR_BYTES = 4
    READ_SENSOR_COMMAND_BASE = 0x50
    READ_TEMP_COMMAND = 0x4E
    READ_HUMIDITY_COMMAND = 0x4F
    READ_PRESSURE_COMMAND = 0x50
    START_MEASUREMENTS_COMMAND = 0x81

    def __init__(self,i2c_address,bus):
        self.bus = bus
        if ((i2c_address >= 1) and (i2c_address <= 0x7f)):
            self.i2c_addr = i2c_address
        else:
            raise RuntimeError("Invalid I2C address (%#0x) creating Fixture object"%(i2c_address))
        self.num_sensor_bytes = 4

    def sendCommand(self,cmd):
        """send a single command byte to the device
        """
        if ((cmd >= 0) and (cmd <= 256)):
            self.bus.write_byte(self.i2c_addr,cmd)

    def readSensor(self,sensor_num):
        """read an individual sensor number and return resistance
           in ohms
        """
        if ((sensor_num < 1) or (sensor_num > 44)):
            sensor_num = 1 #default if out of range

        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.READ_SENSOR_COMMAND_BASE+sensor_num,4)

        return self.bytesToFloat(list_of_bytes)

    def _readI2cData(self,firstFloatCmd,numFloatsToRead, _print=False):
        """
        """
        
        # smbus only supports up to 32-byte reads, so break requests for more than
        # 8 values(4 bytes each) up into 8-value read requests
        return_bytes = []
        while (numFloatsToRead > 0):
            
            if numFloatsToRead > 8:
                bytes_received = self.bus.read_i2c_block_data(self.i2c_addr, firstFloatCmd,
                                                             8*self.NUM_SENSOR_BYTES)
                numFloatsToRead -= 8
                firstFloatCmd += 8
            else:
                bytes_received = self.bus.read_i2c_block_data(self.i2c_addr, firstFloatCmd,
                                                             numFloatsToRead*self.NUM_SENSOR_BYTES)
                numFloatsToRead -= numFloatsToRead
                
            for byte in bytes_received:
                return_bytes.append(byte)
            
        if _print: print return_bytes
        
        float_results = []
        next_sensor_list = []
        byteCounter = 1

        for byte in return_bytes:
            if _print: print byteCounter
            next_sensor_list.append(byte)
            if byteCounter in [1,2,3]:
                byteCounter+=1
            elif byteCounter==4:
                if _print: print next_sensor_list
                float_results.append(self.bytesToFloat(next_sensor_list))
                byteCounter=1
                next_sensor_list = []

        return float_results

    def readSensors(self, startSensor=1, numSensors=22, _print=False):
        """if numSensors not specified: reads all gas concentration sensors on a chip fixture
           if numSensors specified: reads any number of sensor resistances beginning with
           sensor number 1
           Returns sensor readings as a list of floats
        """
        if ( (startSensor < 1) or (startSensor > 44) ):
            startSensor = 1
        if ( (numSensors < 1) or (numSensors > 44) ):
            numSensors = 22
        return self._readI2cData(self.READ_SENSOR_COMMAND_BASE+startSensor,
                              numSensors,_print)

    def readTemperature(self):
        """sends command to read temperature obtained from BME280 sensor
        """
        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.READ_TEMP_COMMAND,4)

        return self.bytesToFloat(list_of_bytes)

    def readHumidity(self):
        """sends command to read humidity obtained from BME280 sensor
        """
        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.READ_HUMIDITY_COMMAND,4)

        return self.bytesToFloat(list_of_bytes)

    def readPressure(self):
        """sends command to read pressure obtained from BME280 sensor
        """
        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.READ_PRESSURE_COMMAND,4)

        return self.bytesToFloat(list_of_bytes)
    
    def readBME280(self):
        """sends command to read temperature,pressure,humidity from BME280 sensor
        """
        return self._readI2cData(self.READ_TEMP_COMMAND,3)

    def startMeasurements(self):
        """sends the START_MEASUREMENTS command over I2C bus
        """
        self.sendCommand(self.START_MEASUREMENTS_COMMAND)

    def readAll(self,totalValues=13):
        """reads pressure,temperature,humidity, and all sensors
        """
        return self._readI2cData(self.READ_TEMP_COMMAND,totalValues)

    def bytesToFloat(self, l):
        """converts a list of 4 bytes to a 32-bit floating point value
        """
        return struct.unpack('!f', self.bytesToHexString(l).decode('hex'))[0]

    def bytesToHexString(self,l):
        """converts a list of 4 bytes (LSB first) to a string of hex characters
           without '0x' at the beginning
        """
        value = l[0] + (l[1]<<8) + (l[2]<<16) + (l[3]<<24) 
        return "{0:0{1}x}".format(value,8)
