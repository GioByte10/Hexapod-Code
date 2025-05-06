import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time


if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_dynamic = CanMotor(can0, motor_id=0, gear_ratio=1)

    motors = [m_dynamic]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")

    m_dynamic.read_status_once()
    m_dynamic.read_multiturn_once()
    m_dynamic.read_motor_state_once()
    m_dynamic.datadump()

    t = float(sys.argv[1])

    m_dynamic.set_control_mode("torque", -t)

    #m_static.control()
    m_dynamic.control()

    try:
        while True:
            print("=============================dynamic=============================")
            m_dynamic.read_status_once()
            m_dynamic.read_multiturn_once()
            m_dynamic.read_motor_state_once()
            m_dynamic.datadump()

            time.sleep(0.07)
            pass

    except KeyboardInterrupt:
        for motor in motors:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
