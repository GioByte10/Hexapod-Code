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

    m_static = CanMotor(can0, motor_id=7, gear_ratio=1)
    m_dynamic = CanMotor(can0, motor_id=0, gear_ratio=1)

    motors = [m_static, m_dynamic]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()


    time.sleep(1)
    input("Continue")


    # m_static.set_control_mode("position", 4.3)
    # m_static.control()
    #
    # m_dynamic.set_control_mode("position", 1.1)
    # m_dynamic.control()

    m_static.read_status_once()
    m_static.read_multiturn_once()
    m_static.read_motor_state_once()
    m_static.datadump()

    m_dynamic.read_status_once()
    m_dynamic.read_multiturn_once()
    m_dynamic.read_motor_state_once()
    m_dynamic.datadump()

    m_static.set_control_mode("torque", 1)
    m_dynamic.set_control_mode("torque", 1)

    m_static.control()
    m_dynamic.control()

    try:
        while True:
            print("=============================static=============================")
            m_static.read_status_once()
            m_static.read_multiturn_once()
            m_static.read_motor_state_once()
            m_static.datadump()

            p_static = m_static.motor_data.singleturn_position + 2 * math.pi - 5.5
            if m_static.motor_data.singleturn_position + 2 * math.pi - 5.4 > 2 * math.pi:
                p_static -= 2 * math.pi

            t_static = p_static - math.pi
            t_static = math.sin(t_static)

            # m_static.set_control_mode("torque", 3 * t_static)
            # m_static.control()

            print(p_static)
            print(t_static)
            print()

            print("=============================dynamic=============================")
            m_dynamic.read_status_once()
            m_dynamic.read_multiturn_once()
            m_dynamic.read_motor_state_once()
            m_dynamic.datadump()

            p_dynamic = m_dynamic.motor_data.singleturn_position + 2 * math.pi - 2.1
            if m_dynamic.motor_data.singleturn_position + 2 * math.pi - 2.03 > 2 * math.pi:
                p_dynamic -= 2 * math.pi

            print(p_dynamic)


            t_dynamic = p_dynamic - math.pi
            t_dynamic = math.sin(t_dynamic)

            # m_dynamic.set_control_mode("torque", 3 * t_dynamic)
            # m_dynamic.control()

            print(t_dynamic)

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
