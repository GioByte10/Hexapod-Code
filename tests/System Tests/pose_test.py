import sys
import os
import time
import can

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper

if __name__ == "__main__":

    # -- INIT CAN INTERFACE --
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    # -- CREATE MOTOR OBJECTS --
    m_D = CanMotor(can0, motor_id=0, gear_ratio=1)  # Dynamic motor
    m_A = CanMotor(can0, motor_id=7, gear_ratio=1)  # Static motor
    motor_list = [m_D, m_A]

    # -- START LISTENER --
    motor_listener = MotorListener(motor_list=motor_list)
    notifier = can.Notifier(can0, [motor_listener])

    # -- INITIALIZE MOTORS --
    for motor in motor_list:
        motor.initialize_motor()

    time.sleep(1)
    input("Continue")

    for motor in motor_list:
        motor.initialize_control_command()

    try:
        while True:
            for motor in motor_list:
                # === UPDATE: Try-except added to prevent crash on CAN overflow ===
                try:
                    motor.read_motor_state_once()  # Lightweight polling
                    motor.datadump()
                except can.CanOperationError as e:
                    print(f"[CAN ERROR] Motor ID {motor.motor_id}: {e}")

            # === UPDATE: Slowed down loop to reduce CAN traffic ===
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")

    finally:
        # === CLEAN SHUTDOWN ===
        for motor in motor_list:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exited cleanly.")
