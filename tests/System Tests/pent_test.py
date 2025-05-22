import sys
import os
import time
import can

# Add parent directory to access the core modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper

if __name__ == "__main__":
    print("[INFO] Initializing CAN interface...")
    core.CANHelper.init("can0")

    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
    print("[INFO] CAN interface ready.")

    # Added custom names for motors to help tell them apart in printouts
    m_D = CanMotor(can0, motor_id=0, gear_ratio=1, name="m_D")
    m_A = CanMotor(can0, motor_id=7, gear_ratio=1, name="m_A")
    motor_list = [m_A, m_D]

    motor_listener = MotorListener(motor_list=motor_list)

    input(">> Press ENTER to start CAN Notifier...")
    notifier = can.Notifier(can0, [motor_listener])
    print("[INFO] CAN Notifier started.")

    input(">> Press ENTER to initialize motors...")
    for motor in motor_list:
        print(f"[INFO] Initializing motor {motor.name} (ID {motor.id})...")
        motor.initialize_motor()


    time.sleep(1)
    input(">> Press ENTER to start periodic sends...")

    for motor in motor_list:
        try:
            # Added this to avoid reinitializing control mode if it's already running
            motor.initialize_control_command()
            # m_D.set_control_mode("speed", 0)
            # m_D.control()

            print(f"[INFO] Control command initialized for {motor.name}.")
        except ValueError as e:
            print(f"[WARNING] {e}")

    print("\n[INFO] Starting motor read loop. Press Ctrl+C to exit.\n")

    try:
        while True:
            for motor in motor_list:
                try:
                    # Added small delays between messages to avoid CAN buffer overflow
                    motor.read_status_once()
                    time.sleep(0.02)

                    motor.read_multiturn_once()
                    time.sleep(0.02)

                    motor.read_motor_state_once()
                    time.sleep(0.02)

                    # Dumps motor state info to the terminal (now labeled by motor name)
                    motor.datadump()

                except OSError as e:
                    # Catch CAN send errors without crashing the script
                    print(f"[ERROR] CAN send error on {motor.name}: {e}")
                    time.sleep(0.1)

            # Extra delay at the end of the loop to reduce bus traffic
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Cleaning up...")
        for motor in motor_list:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()

        # Note: I commented out the line in CANHelper.cleanup that shuts down can0 for debugging
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("[INFO] Shutdown complete. Exiting.")
        exit(0)