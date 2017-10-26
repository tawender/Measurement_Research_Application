#!/usr/bin/python
"""
A Python driver for Alicat mass flow controllers, using serial communication.
Distributed under the GNU General Public License v2
Copyright (C) 2015 NuMat Technologies
"""
import serial
import logging


class FlowMeter(object):
    """Python driver for [Alicat Flow Meters](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-meters/).
    This communicates with the flow meter over a USB or RS-232/RS-485
    connection using pyserial.
    """
    def __init__(self, port='/dev/ttyUSB0', address='A',baud=19200,timeout=1.0):
        """Connects this driver with the appropriate USB / serial port.
        Args:
            port: The serial port. Default '/dev/ttyUSB0'.
            address: The Alicat-specified address, A-Z. Default 'A'.
        """
        self.address = address
        self.timeout = timeout
        logging.info("opening instance of serial port %s(baud:%d,timeout:%.2f) for MFC id: %s"%(
                            port,baud,timeout,self.address))
        try:
            self.connection = serial.Serial(port, baud, timeout=self.timeout)
        except Exception as e:
            logging.error("Couldd not open serial port for MFC addr: %s")%(self.address)
            raise e
        self.keys = ['pressure', 'temperature', 'volumetric_flow', 'mass_flow',
                     'flow_setpoint', 'gas']
        self.gases = ['Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He',
                      'N2', 'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2',
                      'C2H4', 'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10',
                      'C-8', 'C-2', 'C-75', 'A-75', 'A-25', 'A1025', 'Star29',
                      'P-5']

    @classmethod
    def is_connected(cls, port, address='A'):
        """Returns True if the specified port is connected to this device.
        This class can be used to automatically identify ports with connected
        Alicats. Iterate through all connected interfaces, and use this to
        test. Ports that come back True should be valid addresses.
        Note that this distinguishes between `FlowController` and `FlowMeter`.
        """
        is_device = False
        try:
            device = cls(port, address)
            try:
                c = device.get()
                if cls.__name__ == 'FlowMeter':
                    assert c and 'flow_setpoint' not in device.keys
                elif cls.__name__ == 'FlowController':
                    assert c and 'flow_setpoint' in device.keys
                else:
                    raise NotImplementedError('Must be meter or controller.')
                is_device = True
            finally:
                device.close()
        except:
            pass
        return is_device

    def get(self, retries=2):
        """Get the current state of the flow controller.
        From the Alicat mass flow controller documentation, this data is:
         * Pressure (normally in psia)
         * Temperature (normally in C)
         * Volumetric flow (in units specified at time of order)
         * Mass flow (in units specified at time of order)
         * Flow setpoint (only for flow controllers)
         * Total flow (only on models with the optional totalizer function)
         * Currently selected gas
        Args:
            retries: Number of times to re-attempt reading. Default 2.
        Returns:
            The state of the flow controller, as a dictionary.
        """
        try:
            command = '{addr}\r'.format(addr=self.address)
            line = self._write_and_read(command, retries)
            return self._decode_line(line)
        except Exception as e:
            print "failed command: ",command
            raise e
                
    def _decode_line(self, line):
        spl = line.split()
        address, values = spl[0], spl[1:]
        if address != self.address:
            logging.info("Flow controller address mismatch, addr: %s != self.addr: %s"%(address,self.address))
            raise ValueError("Flow controller address mismatch. addr: %s != self.addr: %s"%(address,self.address))
        if len(values) == 5 and len(self.keys) == 6:
            del self.keys[-2]
        elif len(values) == 7 and len(self.keys) == 6:
            self.keys.insert(5, 'total flow')
        return {k: (v if k == self.keys[-1] else float(v))
                for k, v in zip(self.keys, values)}

    def set_gas(self, gas, retries=2):
        """Sets the gas type.
        Args:
            gas: The gas type, as a string. Supported gas types are:
                'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He', 'N2',
                'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2', 'C2H4',
                'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2',
                'C-75', 'A-75', 'A-25', 'A1025', 'Star29', 'P-5'
        """
        if gas not in self.gases:
            logging.error("Attempted to set unsupported gas type: %s"%(gas))
            raise ValueError("{} not supported!".format(gas))
        command = '{addr}$${gas}\r'.format(addr=self.address,
                                           gas=self.gases.index(gas))
        line = self._write_and_read(command, retries)
        if line.split()[-1] != gas:
            raise IOError("Could not set gas type")

    def flush(self):
        """Reads all available information. Use to clear queue."""
        self.connection.flush()
        self.connection.flushInput()
        self.connection.flushOutput()

    def close(self):
        """Closes the serial port. Call this on program termination."""
        self.flush()
        logging.info("Closing serial connection to MFC addr %s"%(self.address))
        self.connection.close()

    def _write_and_read(self, command, retries=2):
        """Writes a command and reads a response from the flow controller."""
        for _ in range(retries+1):
            logging.info("writing command(MFC addr %s):  %s"%(self.address,command))
            self.connection.write(command.encode('utf-8'))
            line = self._readline()
            if line:
                logging.info("Received: %s"%(line))
                return line                
            else:
                logging.error("No line read from flow controller(MFC addr %s)"%
                                            (self.address))
                raise IOError("Could not read from flow controller.")

    def _readline(self):
        """Reads a line using a custom newline character (CR in this case).
        Function from http://stackoverflow.com/questions/16470903/
        pyserial-2-6-specify-end-of-line-in-readline
        """
        line = bytearray()
        while True:
            try:
                c = self.connection.read(1)
            except Exception as e:
                logging.error("Exception reading character from MFC addr: %s")%(self.address)
                raise e
            if c:
                line += c
                if line[-1] == ord('\r'):
                    #check for a returned line consisting of a single number
                    if len(line.strip().split()) == 1:
                        print "Getting rid of this number"
                        logging.info("Discarding line with single setpoint returned: %s"%(line.decode('utf-8').strip()))
                        line = bytearray()
                    else:
                        break
            else:
                break
        try:
            line_elements = line.decode('utf-8').strip()
            return line_elements
        except Exception as e:
            logging.error("Exception in [MFC addr: %s]_readline() decoding line: "%(self.address),line)
            raise e

class FlowController(FlowMeter):
    """Python driver for [Alicat Flow Controllers](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-controllers/).
    This communicates with the flow controller over a USB or RS-232/RS-485
    connection using pyserial.
    To set up your Alicat flow controller, power on the device and make sure
    that the "Input" option is set to "Serial".
    """
    def set_flow_rate(self, flow, retries=2, verify_flow_change=True):
        """Sets the target flow rate.
        Args:
            flow: The target flow rate, in units specified at time of purchase
        """
        command = '{addr}S{flow:.2f}\r'.format(addr=self.address, flow=flow)
        line = self._write_and_read(command, retries)
        
        if line[0] != self.address:
            logging.error("Expected address(%s) not found in returned line: %s"%(
                                self.address,line))
            #raise an error here
            return
            
        if verify_flow_change:
            #convert line into dictionary of returned data
            returned_values = self._decode_line(line)
            
            if abs(float(returned_values['flow_setpoint']) - flow) > 0.01:
                logging.error("device response[%s] does not match flow rate that was set[%.2f]"%
                                    (returned_values['flow_setpoint'],flow))
                raise IOError("Could not set flow.")
                    
        


def command_line():
    import argparse
    import json
    from time import time

    parser = argparse.ArgumentParser(description="Control an Alicat mass "
                                     "flow controller from the command line.")
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0', help="The "
                        "target serial or USB port. Default /dev/ttyUSB0.")
    parser.add_argument('--address', '-a', default='A', type=str, help="The "
                        "device address, A-D. Should only be used if multiple "
                        "flow controllers are connected to one port.")
    parser.add_argument('--set-gas', '-g', default=None, type=str,
                        help="Sets the gas type. Supported gas types are: "
                             "'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', "
                             "'He', 'N2', 'N2O', 'Ne', 'O2', 'C3H8', "
                             "'n-C4H10', 'C2H2', 'C2H4', 'i-C2H10', 'Kr', "
                             "'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2', "
                             "'C-75', 'A-75', 'A-25', 'A1025', 'Star29', "
                             "'P-5'")
    parser.add_argument('--set-flow-rate', '-f', default=None, type=float,
                        help="Sets the target flow rate of the controller.")
    parser.add_argument('--stream', '-s', action='store_true',
                        help="Sends a constant stream of flow controller "
                             "data, formatted as a tab-separated table.")
    args = parser.parse_args()

    flow_controller = FlowController(port=args.port, address=args.address)

    state = flow_controller.get()
    is_controller = ('flow_setpoint' in flow_controller.keys)
    if args.set_gas:
        flow_controller.set_gas(args.set_gas)
    if args.set_flow_rate is not None:
        if is_controller:
            flow_controller.set_flow_rate(args.set_flow_rate)
        else:
            raise TypeError("Cannot set flow on meter.")

    if args.stream:
        try:
            print('time\t' + '\t'.join(flow_controller.keys))
            t0 = time()
            while True:
                state = flow_controller.get()
                print('{:.2f}\t'.format(time() - t0) +
                      '\t\t'.join('{:.2f}'.format(state[key])
                                  for key in flow_controller.keys[:-1]) +
                      '\t\t' + state['gas'])
        except KeyboardInterrupt:
            pass
    else:
        print(json.dumps(state, indent=2, sort_keys=True))
    flow_controller.close()


if __name__ == '__main__':
    command_line()
