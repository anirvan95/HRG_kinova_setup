import serial
import threading
import struct
import logging


class FSRray:
    def __init__(self, width=16, verbose=False):
        self.array_width = width
        self._callback = None
        self._values = [0] * width * width
        self._dt = [0] * 2
        self._path = None
        self._baud = 500000
        self._timeout = 3
        self._thread = None
        self._running = False
        self._verbose = verbose
        logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO)
        logging.info("FSRray initialized")

    def set_callback(self, callback):
        self._callback = callback

    def connect(self, path="/dev/ttyACM0", baud=500000, timeout=3):
        self._path = path
        self._baud = baud
        self._timeout = timeout
        self._thread = threading.Thread(target=self.run)
        self._thread.start()

    def disconnect(self):
        self._running = False
        self._thread.join()

    def run(self):
        with serial.Serial(self._path, self._baud, timeout=self._timeout) as arduino:
            d = b'\x00'
            dt = [0, 0]
            self._running = True
            time.sleep(1)
            while (self._running):
                n = arduino.write(bytearray([self.array_width]))  # send array width
                for i in range(2):  # read two timestamps on 4 bytes each
                    self._dt[i] = struct.unpack('<I', arduino.read(4))[0]
                    logging.debug("dt[{}] = {}".format(i, self._dt[i]))
                for i in range(self.array_width * self.array_width):  # read the n*n values on 2 bytes each
                    self._values[i] = struct.unpack('<H', arduino.read(2))[0]
                    logging.debug("values[{}] = {}".format(i, self._values[i]))
                if self._callback:
                    self._callback(self._values, self._dt)


if __name__ == "__main__":
    import time


    def callback(values, dt):
        print("dt = {}".format(dt[0]))
        print("values = {}".format(values))


    fsrray = FSRray()
    fsrray.set_callback(callback)
    fsrray.connect()
    #time.sleep()
    #fsrray.disconnect()