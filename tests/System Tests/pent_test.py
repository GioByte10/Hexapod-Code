import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
import numpy as np
import math


if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    # motor = CanMotor(can0, motor_id=7, gear_ratio=1) #m_A
    m_A = CanMotor(can0, MAX_SPEED=7, motor_id=0, gear_ratio=1)
    m_D = CanMotor(can0, MAX_SPEED=0, motor_id=0, gear_ratio=1)
    motors = [m_A, m_D]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])


    # motor.write_pid(0x01, 1000.0) #
    # motor.write_pid(0x02, 1)
    #
    # motor.write_pid(0x04, 1000.0)
    # motor.write_pid(0x05, 1)
    #
    # motor.write_pid(0x07, np.float32(1000.1)) # KP
    # motor.write_pid(0x08, 1)
    # motor.write_pid(0x09, 1)


    # motor.write_acceleration(0x00, np.uint32(0))
    # motor.write_acceleration(0x01, np.uint32(0))

    # motor.write_acceleration(0x02, np.uint32(60000))
    # motor.write_acceleration(0x03, np.uint32(60000))

    for motor in motors:
        motor.initialize_motor()

    time.sleep(1)
    input("Continue")

    m_A.initialize_control_command()
    m_A.set_control_mode("torque", 10)
    m_A.control()

    m_D.initialize_control_command()
    m_D.set_control_mode("torque", 10)
    m_D.control()

    t = 0

    try:
        while True:
            pass
            # motor.read_status_once()
            # time.sleep(0.02)
            # motor.read_multiturn_once()
            # time.sleep(0.02)
            # motor.read_motor_state_once()
            # time.sleep(0.02)
            # #motor.read_pid_once()
            # time.sleep(0.02)
            # motor.datadump()

            # motor.set_control_mode("position", 3 + math.sin(t))
            # print(math.sin(t))


    except KeyboardInterrupt:
        for motor in motors:
            motor.stop_all_tasks()
            motor.motor_off()
        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
