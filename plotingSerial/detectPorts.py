from serial.tools import list_ports

def get_available_ports():
    ports = list_ports.comports()
    return [port.device for port in ports]

ports = get_available_ports()

for port in ports:
    print(port)

#print(ports)