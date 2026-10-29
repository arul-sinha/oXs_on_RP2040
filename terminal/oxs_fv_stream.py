import serial
import time
import math
# import csv


PORT='COM7'

def calibrate():
    # FORMAT THE READINGS SOMEHOW
    input("Remove all weights from the device. Press Enter")
    time.sleep(2)
    zero_reading = format(fetchReadings())

    readings = []
    knownMass = eval(input("Enter mass of object in grams: "))
    knownMass /= 1000   # convert to KGs
    force = 9.81 * pow(6371/(6371+(984/1000)),2) * knownMass

    print(zero_reading, force)
    for i in range(5):
 
        readings.append(format(fetchReadings()))
        print("Remove, Keep it again")
        time.sleep(2)

    for i in readings:
        if math.fabs(i - math.fabs(sum(readings)/len(readings))) >=2000:
            readings.remove(i)

    c = (math.fabs(sum(readings)/len(readings)) - zero_reading) / force
    print(readings)
    return math.fabs(c)    

# readings = []

def fetchReadings():   
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
        
        pico.write(b"FV\r\n")
        pico.flush()
    
        response = pico.readall().decode(
            "utf-8", errors="replace"
        ).strip()

        if response:
            # readings.append(response)
            # print("Received:", response)
            pass
        else:
            print("no response")

    return response

def format(response):
    response = (response.split("\n")[4])
    response = response.replace("Reserve 6 = ", "")
    response = int(response)
    return response

#print(calibrate())

while True:
    print(fetchReadings())
    time.sleep(.1)

# 5838.810218638676
# 5675.555013777419
# 5747.220419550397



