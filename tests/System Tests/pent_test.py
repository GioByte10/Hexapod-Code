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

    motor = CanMotor(can0, motor_id=0, gear_ratio=1)
    motor_list = [motor]
    motor_listener = MotorListener(motor_list=motor_list)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motor_list:
        motor.initialize_motor()

    time.sleep(1)
    input("Continue")

    motor.initialize_control_command()
    # motor.set_control_mode("torque", 0)
    # motor.control()

    try:
        while True:
            motor.read_status_once()
            motor.read_multiturn_once()
            motor.read_motor_state_once()
            motor.datadump()

            time.sleep(0.05)
            pass

    except KeyboardInterrupt:
        motor.stop_all_tasks()
        motor.motor_off()
        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
