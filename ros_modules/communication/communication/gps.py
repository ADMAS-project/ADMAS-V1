###### reading and processing GPS data ############
####### now we from gps import GPS ############

#!/usr/bin/env python3
import serial
import pynmea2
import time

class GPS:
    def __init__(self, port='/dev/ttyAMA0', baud=9600, timeout=1):
        self.serial = None
        try:
            self.serial = serial.Serial(port, baud, timeout)
        except serial.SerialException as e:
            raise RuntimeError(f"Serial failed: {e}")

    def read(self):
        try:
            line = self.serial.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                if (msg.latitude is not None and msg.longitude is not None and 
                    msg.gps_qual > 0 and 
                    not (msg.latitude == 0.0 and msg.longitude == 0.0)):
                    return {
                        "lat": msg.latitude,
                        "lon": msg.longitude,
                        "time": time.time()
                    }
            return None
        except (serial.SerialException, pynmea2.ParseError) as e:
            raise RuntimeError(f"GPS error: {e}")

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()