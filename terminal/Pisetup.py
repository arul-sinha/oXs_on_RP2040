import serial
import time
PORT='COM7'

def setupPI():   
    #print("Attempting to fetch readings...")

    with serial.Serial(
        port=PORT,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    ) as pico:

        time.sleep(0.5)  
        
        pico.write(b"PROTOCOL=S\r\n")
        pico.flush()

        pico.write(b"PRI=5\r\n")
        pico.flush()

        pico.write(b"SDA=14\r\n")
        pico.flush()

        pico.write(b"SCL=15\r\n")
        pico.flush()

        pico.write(b"TLM=18\r\n")
        pico.flush()

        pico.write(b"SAVE\r\n")
        pico.flush()
    pass


setupPI()