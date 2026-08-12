import serial

class Transport:
    'Carries lines of text. Real serial or fake- callers cant taell.'
    def write_line(self, line): raise NotImplementedError
    def read_line(self, timeout=None): raise NotImplementedError
    
    
class LoopbackEndpoint(Transport):
    def __init__(self, rx, tx):
        self._rx = rx
        self._tx = tx
        
        
    def write_line(self, line): 
        self._tx.append(line)
        
    
    
    def read_line(self, timeout=None):
        if not self._rx:
            return None
        return self._rx.pop(0)
        
        
def loopback_pair():
    a2b = []
    b2a = []
    laptop = LoopbackEndpoint(rx=b2a, tx=a2b)
    device = LoopbackEndpoint(rx=a2b, tx = b2a)
    return laptop, device


class RealSerial(Transport):
    def __init__(self, port, baud = 115200, timeout=1.0):
        if port.startswith("sim://"):
            from station.fake_port import FakeSerialPort
            self._ser = FakeSerialPort(port)
        else:
            self._ser = serial.serial_for_url(port, baudrate=baud, timeout=timeout)
            


    def write_line(self, line):
        current = line + "\n"
        
        current = current.encode() 
        self._ser.write(current)
        
    def read_line(self, timeout=None):
        raw = self._ser.readline()
        
        if not raw:
            return None
        
        else:
            text = raw.decode()
            
            return text.strip()


        

