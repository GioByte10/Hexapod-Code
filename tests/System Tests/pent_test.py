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

    # motor = CanMotor(can0, motor_id=7, gear_ratio=1) #m_A
    motor = CanMotor(can0, motor_id=0, gear_ratio=1)   #m_D
    motors = [motor]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    motor.write_pid(0x07, 1000.0)
    motor.write_pid(0x07, 1)

    for motor in motors:
        motor.initialize_motor()


    time.sleep(1)
    input("Continue")

    motor.initialize_control_command()
    motor.set_control_mode("position", 1.0)
    motor.control()

    try:
        while True:
            motor.read_status_once()
            time.sleep(0.02)
            motor.read_multiturn_once()
            time.sleep(0.02)
            motor.read_motor_state_once()
            time.sleep(0.02)
            #motor.read_pid_once()
            time.sleep(0.02)
            motor.datadump()

            time.sleep(0.05)
            motor.control()

    except KeyboardInterrupt:
        motor.stop_all_tasks()
        motor.motor_off()
        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
