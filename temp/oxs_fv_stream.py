import serial
import time
import math
# import csv


PORT='COM3'


def calibrate():
    # FORMAT THE READINGS SOMEHOW
    input("Remove all weights from the device. Press Enter")
    time.sleep(2)
    zero_reading = fetchReadings(PORT)

    readings = []
    knownMass = eval(input("Enter mass of object in grams: "))
    knownMass /= 1000   # convert to KGs
    force = 9.81 * pow(6371/(6371+(984/1000)),2) * knownMass
    for i in range(5):
 
        readings.append(fetchReadings(PORT))
        print("Keep it again")
        time.sleep(2)

    c = (math.fabs(sum(readings)/len(readings)) - zero_reading) / force

    return c

# readings = []

def fetchReadings(PORT):   
    print("Attempting to fetch readings...")

    with serial.Serial(
        port=PORT,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=2
    ) as pico:

        time.sleep(2)  

        pico.write(b"FV\r\n")
        pico.flush()

        response = pico.readline().decode(
            "utf-8", errors="replace"
        ).strip()

        if response:
            # readings.append(response)
            print("Received:", response)
        else:
            print("no response")

    return response