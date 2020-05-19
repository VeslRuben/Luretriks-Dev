
import time
from roboclaw_3 import Roboclaw


class Swag:
    def __init__(self):
        self.rc = Roboclaw("/dev/ttyACM0", 115200)  # Linux comport name
        self.address = 0x80
        self.rc.Open()
        self.ready = True
        version = self.rc.ReadVersion(self.address)
        if not version[0]:
            print("GETVERSION Failed")
            exit()
        else:
            print(repr(version[1]))
            print("Car main battery voltage at start of script: ", self.rc.ReadMainBatteryVoltage(self.address))
        for i in range(1000):
            try:
                self.rc.ForwardM2(self.address, i)
                time.sleep(0.1)
                print(i)
            except Exception as e:
                print(e)


    def control_speed(self, mc, adr, speed_m1, speed_m2):
        # speedM1 = leftMotorSpeed, speedM2 = rightMotorSpeed
        if speed_m1 > 0:
            mc.ForwardM1(adr, speed_m1)
        elif speed_m1 < 0:
            speed_m1 = speed_m1 * (-1)
            mc.BackwardM1(adr, speed_m1)
        else:
            mc.ForwardM1(adr, 0)

        if speed_m2 > 0:
            mc.ForwardM2(adr, speed_m2)
        elif speed_m2 < 0:
            speed_m2 = speed_m2 * (-1)
            mc.BackwardM2(adr, speed_m2)
        else:
            mc.ForwardM2(adr, 0)


if __name__ == "__main__":
    t = Swag()
