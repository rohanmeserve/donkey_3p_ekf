import socket

class ATLAS_GNSS():
    '''
    A modified gps part that establishes a connection to an ATLUS GNSS system via UDP
    '''

    def __init__(self):
        # stuff
        UDP_IP = "192.168.111.200"
        UDP_PORT = 23452
        self.sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
        print('attempting to connect')
        self.sock.bind(('', UDP_PORT))
        self.sock.connect((UDP_IP, UDP_PORT))
        print('connection established')

    def run(self):
        # receive packets from the established connection
        # should be raw nmea sentences
        nmea_sent = self.sock.recv(1024)
        return nmea_sent