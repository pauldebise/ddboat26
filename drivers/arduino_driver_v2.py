import serial
import time
import signal
import sys

class ArduinoIO():
    def __init__(self, calibrate=True):
        self.arduino = None
        self.cmdl = 0
        self.cmdr = 0
        self.dirl = ' '
        self.dirr = ' '
        print ("Init Arduino ...")
        init_arduino_tty = False
        try:
            self.arduino = serial.Serial('/dev/ttyV0',115200,timeout=1.0)
            #self.arduino = serial.Serial('/dev/ttyACM0V0',115200,timeout=1.0)
            data = self.arduino.readline()
            print ("init status : ",data.decode("utf-8")[0:-1])
            init_arduino_tty = True
            calibrate = False # Skip calibration (done at boot)
        except:
            print ("Cannot initialize Arduino driver (virtual tty)")

        if not init_arduino_tty:
            try:
                self.arduino = serial.Serial('/dev/ttyACM0',115200,timeout=10.0)
                data = self.arduino.readline()
                print ("init status : ",data.decode("utf-8")[0:-1])
                init_arduino_tty = True
            except:
                print ("Cannot initialize Arduino driver")

        signal.signal(signal.SIGINT, self.signal_handler)
        if calibrate:
            print ("Calibrating ESC ...")
            print ("ESC status",self.calibrate_esc())
        else:
            print ("ESC calibration skipped, assuming already done!")
        print ("Arduino OK ...")
    
    def signal_handler(self,sig, frame):
        print('You pressed Ctrl+C! set motors to 0!')
        #serial_arduino = serial.Serial('/dev/ttyACM0',115200,timeout=1.0)
        data = self.send_arduino_cmd_motor(0,0)
        sys.exit(0)

    def bound_cmd (self,cmd0):
        cmd = cmd0
        if cmd > 255:
            cmd = 255
        if cmd < -255:
            cmd = -255
        return cmd
    
    def calibrate_esc (self,timeout=1.0):
        strcmd = "I;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        time.sleep(7.0)
        t0 = time.time()
        while True:
            data = self.arduino.readline()
            if data:
                #print (data.decode())
                break
            if (time.time()-t0) > timeout:
                print ("calibrate_esc timeout",timeout)
                break
        return data.decode("utf-8")[0:-1]
    
    def send_arduino_cmd_motor(self,cmdl0,cmdr0):    
        self.cmdl = self.bound_cmd(cmdl0)
        self.cmdr = self.bound_cmd(cmdr0)
        self.dirl = ' '
        self.dirr = ' '
        # on new BBBOATs backwards allowed !!
        if self.cmdl < 0:     
            self.dirl = '-'
            self.cmdl = -self.cmdl
        if self.cmdr < 0:
            self.dirr = '-'
            self.cmdr = -self.cmdr
        #self.cmdl = abs(self.cmdl)  # no need to abs on new BBBOATs (backwards allowed)
        #self.cmdr = abs(self.cmdr)
        #print ("\n",self.dirl,self.cmdl,self.dirr,self.cmdr)
        strcmd = "M%c%3.3d%c%3.3d;"%(self.dirl,self.cmdl,self.dirr,self.cmdr)
        #print (strcmd+"\n")
        self.arduino.write(strcmd.encode())

    def get_arduino_cmd_motor(self,timeout=1.0):    # set by RC command ?
        strcmd = "C;"
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        data="\n"
        while True:
            data = self.arduino.readline()
            if data:
                #print (data)
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_cmd_motor timeout",timeout)
                break
        return data.decode("utf-8")[0:-1]

    def get_arduino_cmd_motor_esc(self,timeout=1.0):    
        strcmd = "Z;"
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        data="\n"
        while True:
            data = self.arduino.readline()
            if data:
                #print (data)
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_cmd_motor timeout",timeout)
                break
        return data.decode("utf-8")[0:-1]

    # last motor command sent 
    def get_cmd_motor(self):
        return self.cmdl,self.cmdr

    def get_arduino_rc_chan(self,timeout=1.0):
        strcmd = "R;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        data = "\n"
        while True:
            data = self.arduino.readline()
            if data:
                #print (data)
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_rc_chan timeout",timeout)
                break
        return data.decode("utf-8")[0:-1]

    def get_arduino_raw_rc_chan(self,timeout=1.0):
        strcmd = "X;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        data = "\n"
        while True:
            data = self.arduino.readline()
            if data:
                #print (data)
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_rc_chan timeout",timeout)
                break
        return data.decode("utf-8")[0:-1]
   
    def get_arduino_status(self,timeout=1.0):
        strcmd = "S;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        while True:
            data = self.arduino.readline()
            if data:
                #print (data.decode())
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_status timeout",timeout)
                break
        return data.decode("utf-8")[0:-1]
   
    def get_arduino_energy_saver(self,timeout=1.0):
        strcmd = "E;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        while True:
            data = self.arduino.readline()
            if data:
                #print (data.decode())
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_status timeout",timeout)
                break
        return data.decode("utf-8")[0:-1]

if __name__ == "__main__":
    ard = ArduinoIO()
    try:
        cmdl = int(sys.argv[1])
    except:
        cmdl = 30
    try:
        cmdr = int(sys.argv[2])
    except:
        cmdr = 30

    ard.send_arduino_cmd_motor (cmdl,cmdr)
    print ("Arduino status is",ard.get_arduino_status())
    print ("cmd motors l,r are",ard.get_cmd_motor())
    print ("RC channel is",ard.get_arduino_rc_chan())
    
    print ("Hit Ctrl+C to get out!")
    while True:
        print ("arduino motors rc",ard.get_arduino_cmd_motor())
        print ("arduino ESC ",ard.get_arduino_cmd_motor_esc())
        print ("RC channel is",ard.get_arduino_rc_chan())
        print ("ENSV: ",ard.get_arduino_energy_saver())

        time.sleep(1.0)
