import struct


class ChipFixture(object):
    
    NUM_SENSOR_BYTES = 4
    
    CMD_READ_SENSOR_BASE = 0x50
    CMD_SELECT_SENSOR_BASE = 0xB0
    CMD_READ_TEMPERATURE = 0x4E
    CMD_READ_HUMIDITY = 0x4F
    CMD_READ_PRESSURE = 0x50
    CMD_START_MEASUREMENTS = 0x81
    CMD_ENABLE_AUTORANGE		= 0x1F
    CMD_DISABLE_AUTORANGE		= 0x20
    CMD_SET_DAC1 			= 0x21
    CMD_SET_DAC2 			= 0x22
    CMD_SELECT_CONST_VOLTAGE_CIRCUIT 	= 0x27
    CMD_SELECT_CONST_CURRENT_CIRCUIT 	= 0x28
    CMD_GROUND_CC_CIRCUIT 		= 0x29
    CMD_UNGROUND_CC_CIRCUIT 		= 0x2A
    CMD_GROUND_SENSORS			= 0x2B
    CMD_UNGROUND_SENSORS		= 0x2C
    CMD_MEASURE_ADC16			= 0x2D
    CMD_MEASURE_SENSOR			= 0x2E
    CMD_SELECT_RBIAS1			= 0x30
    CMD_SELECT_RBIAS2			= 0x31
    CMD_SELECT_RBIAS3			= 0x32
    CMD_SELECT_RBIAS4			= 0x33
    
    DUMMY_SENSORS = [1.0,1.5,2.0,2.7,3.3,3.9,4.7,5.6,6.2,6.81,7.5,
			8.2,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.4,
			18.0,20.0]

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
	    
    def setDAC(self,DACnum,voltage):
	"""send command, then data to change DAC setting"""
	dac_counts = int(voltage * 4095 / 2.5)
	if DACnum == 1:
	    self.sendCommand(self.CMD_SET_DAC1)
	elif DACnum == 2:
	    self.sendCommand(self.CMD_SET_DAC2)
	else:
	    print "Invalid DAC number"
	    return
	self.bus.write_byte(self.i2c_addr, dac_counts >> 8)
	self.bus.write_byte(self.i2c_addr, dac_counts & 0x00FF)  

    def selectSensor(self,sensorNum):
	"""send command to select a certain sensor on the measurement pcb mux"""
	self.sendCommand(self.CMD_SELECT_SENSOR_BASE + sensorNum)
	
    def measureADC16(self):
	"""send command for 16-bit ADC to measure voltage level"""
	self.sendCommand(self.CMD_MEASURE_ADC16)
	
    def measureSelectedSensor(self):
	"""send command to measure the currently selected sensor
	   and store the result for manual reading using MSP430 debugger"""
	self.sendCommand(self.CMD_MEASURE_SENSOR)

    def selectConstantVoltage(self):
	"""send command to select constant voltage measurement circuit"""
	self.sendCommand(self.CMD_SELECT_CONST_VOLTAGE_CIRCUIT)

    def selectConstantCurrent(self):
	"""send command to select constant current measurement circuit"""
	self.sendCommand(self.CMD_SELECT_CONST_CURRENT_CIRCUIT)
	
    def selectRbias(self,rNum):
	"""select the bias resistor for the constant current circuit (1-4)"""
	if (rNum==1):
	    self.sendCommand(self.CMD_SELECT_RBIAS1)
	elif (rNum==2):
	    self.sendCommand(self.CMD_SELECT_RBIAS2)
	elif (rNum==3):
	    self.sendCommand(self.CMD_SELECT_RBIAS3)
	elif (rNum==4):
	    self.sendCommand(self.CMD_SELECT_RBIAS4)

    def enableAutorange(self):
	"""send command to enable autorange function when constant current circuit in use"""
	self.sendCommand(self.CMD_ENABLE_AUTORANGE)

    def disableAutorange(self):
	"""send command to disable autorange function when constant current circuit in use"""
	self.sendCommand(self.CMD_DISABLE_AUTORANGE)

    def groundSensors(self):
	"""send command to close the switch grounding one side of all sensor locations"""
	self.sendCommand(self.CMD_GROUND_SENSORS)

    def ungroundSensors(self):
	"""send command to open the switch grounding one side of all sensor locations"""
	self.sendCommand(self.CMD_UNGROUND_SENSORS)

    def groundCCcircuit(self):
	"""send command to close switch U9 grounding the constant current circuit"""
	self.sendCommand(self.CMD_GROUND_CC_CIRCUIT)

    def ungroundCCcircuit(self):
	"""send command to open switch U9 grounding the constant current circuit"""
	self.sendCommand(self.CMD_UNGROUND_CC_CIRCUIT)

    def readSensor(self,sensor_num):
        """read an individual sensor number and return resistance
           in ohms
        """
        if ((sensor_num < 1) or (sensor_num > 44)):
            sensor_num = 1 #default if out of range

        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.CMD_READ_SENSOR_BASE+sensor_num,4)

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
        return self._readI2cData(self.CMD_READ_SENSOR_BASE+startSensor,
                              numSensors,_print)

    def readTemperature(self):
        """sends command to read temperature obtained from BME280 sensor
        """
        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.CMD_READ_TEMPERATURE,4)

        return self.bytesToFloat(list_of_bytes)

    def readHumidity(self):
        """sends command to read humidity obtained from BME280 sensor
        """
        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.CMD_READ_HUMIDITY,4)

        return self.bytesToFloat(list_of_bytes)

    def readPressure(self):
        """sends command to read pressure obtained from BME280 sensor
        """
        list_of_bytes = self.bus.read_i2c_block_data(self.i2c_addr,self.CMD_READ_PRESSURE,4)

        return self.bytesToFloat(list_of_bytes)
    
    def readBME280(self):
        """sends command to read temperature,pressure,humidity from BME280 sensor
        """
        return self._readI2cData(self.CMD_READ_TEMPERATURE,3)

    def startMeasurements(self):
        """sends the START_MEASUREMENTS command over I2C bus
        """
        self.sendCommand(self.CMD_START_MEASUREMENTS)

    def readAll(self,totalValues=13):
        """reads pressure,temperature,humidity, and all sensors
        """
        return self._readI2cData(self.CMD_READ_TEMPERATURE,totalValues)

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
	
    def testAll(self,numSensorsToRead=25):
	sensornum = 1
	r = self.readAll(numSensorsToRead)
	
	print "\n***************************************************************************"
	print "      Temp: %.2f*C  RH: %.2f%%  Pres: %.0fPa"%(
		    r[0],r[1],r[2])
	
	numRows = (numSensorsToRead-3)/2
	for i in range(numRows):
	    print "  Sensor%2d: %.0fohms  err: %4.1f%%"%(sensornum,r[sensornum+2],
			    (r[sensornum+2]-(self.DUMMY_SENSORS[sensornum-1]*1000))/(self.DUMMY_SENSORS[sensornum-1]*1000)*100),
	    sensornum += 1
	    if sensornum <= numSensorsToRead-3:
		print "\tSensor%2d: %.0fohms  err: %4.1f%%"%(sensornum,r[sensornum+2],
			    (r[sensornum+2]-(self.DUMMY_SENSORS[sensornum-1]*1000))/(self.DUMMY_SENSORS[sensornum-1]*1000)*100),
		sensornum += 1
	    print
	print "***************************************************************************\n"
	
