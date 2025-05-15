import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time

def end():
    for motor in motors:
        motor.stop_all_tasks()
        motor.motor_off()

    notifier.stop()
    core.CANHelper.cleanup("can0")
    can0.shutdown()
    print("Exiting")
    exit(0)


if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_dynamic = CanMotor(can0, motor_id=0, gear_ratio=1)
    m_static = CanMotor(can0, motor_id=7, gear_ratio=1)

    motors = [m_dynamic, m_static]
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

    m_static.read_status_once()
    m_static.read_multiturn_once()
    m_static.read_motor_state_once()
    m_static.datadump()

    t = float(sys.argv[1])

    if len(sys.argv) == 3:
        s = int(sys.argv[2])

    else:
        s = 1

    m_dynamic.set_control_mode("torque", -t)
    m_static.set_control_mode("torque", -t)

    m_static.control()
    m_dynamic.control()

    start_time = time.time()

    try:
        while time.time() - start_time < s:
            print("=============================dynamic=============================")
            m_dynamic.read_status_once()
            m_dynamic.read_multiturn_once()
            m_dynamic.read_motor_state_once()
            m_dynamic.datadump()

            m_dynamic.control()

            print("=============================static=============================")
            m_static.read_status_once()
            m_static.read_multiturn_once()
            m_static.read_motor_state_once()
            m_static.datadump()
            
            m_static.control()

            time.sleep(0.07)
            pass

        end()

    except KeyboardInterrupt:
        end()
