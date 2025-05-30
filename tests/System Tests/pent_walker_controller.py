import sys
import os
import threading
import time
from evdev import InputDevice, categorize, ecodes, list_devices

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
import numpy as np
import math
import scipy.io

A_OFFSET = 4.654 - 2 * math.pi
D_OFFSET = 1.769
RANGE = 10
TOP_N = 3

changed_file_n = False
changed_control_mode = False
restart = False

lock = threading.Lock()

control_indexes = {
    "p": 1,
    "v": 2,
    "s": 3
}

control_modes = {
    1: "position",
    2: "speed",
    3: "shape"
}

def find_controller():
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        print(f"Found: {device.name} at {device.path}")
        if 'Xbox' in device.name or 'xbox' in device.name:
            print("Using:", device.path)
            return device

    print("ERROR: Controller not connected")
    exit(3)


def read_controller_inputs(device):
    global file_n, changed_file_n, control_index, changed_control_mode, restart

    for event in device.read_loop():
        if event.type == ecodes.EV_ABS and event.value:

            if event.code == ecodes.ABS_HAT0Y:
                with lock:
                    changed_file_n = True
                    file_n -= event.value

                    if file_n < 1:
                        file_n += 3

                    elif file_n > 3:
                        file_n -= 3

            elif event.code == ecodes.ABS_HAT0X:
                with lock:
                    changed_control_mode = True
                    control_index += event.value

                    if control_index < 1:
                        control_index += len(control_indexes)

                    elif control_index > len(control_indexes):
                        control_index -= len(control_indexes)

        elif event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            if key_event.keystate == key_event.key_down and 'BTN_Y' in key_event.keycode:
                with lock:
                    restart = True

def noop(*args, **kwargs):
    pass


def no_control():
    print("No control mode")


def end():
    print("Ending")
    set_initial_position(A_OFFSET, D_OFFSET)

    for motor in motors:
        motor.stop_all_tasks()
        motor.motor_off()

    notifier.stop()
    core.CANHelper.cleanup("can0")
    can0.shutdown()
    print("Exiting")
    exit(0)


def restart_motors():
    for motor in motors:
        motor.stop_all_tasks()

        motor.write_acceleration(0x00, np.uint32(60000))
        motor.write_acceleration(0x01, np.uint32(60000))

        motor.write_acceleration(0x02, np.uint32(100))
        motor.write_acceleration(0x03, np.uint32(100))

        motor.initialize_motor()
        motor.initialize_control_command()


def parse_arguments():
    debug = False
    it = 0

    if len(sys.argv) >= 3:
        file_n = int(sys.argv[1])
        control_index = control_indexes[sys.argv[2]]

    else:
        print("ERROR: Filename or control mode not provided")
        exit(1)

    if len(sys.argv) >= 4:
        debug = "debug" in sys.argv

        for i in range(3, len(sys.argv)):
            try:
                it = int(sys.argv[i])
            except ValueError:
                it = 0

    return file_n, control_index, debug, it


def datadump():
    for motor in motors:
        motor.read_status_once()
        time.sleep(0.02)
        motor.read_multiturn_once()
        time.sleep(0.02)
        motor.read_motor_state_once()
        time.sleep(0.02)

    for motor in motors:
        motor.datadump()
        time.sleep(0.02)


def set_initial_position(p_A, p_D):
    m_A.set_control_mode("position", p_A)
    m_D.set_control_mode("position", p_D)

    m_A.control()
    m_D.control()

    time.sleep(0.2)

    print(f'p_A = {p_A}')
    print(f'p_D = {p_D}')

    while True:
        m_A.read_motor_state_once()
        time.sleep(0.02)
        m_A.read_multiturn_once()
        time.sleep(0.02)

        m_D.read_motor_state_once()
        time.sleep(0.02)
        m_D.read_multiturn_once()
        time.sleep(0.02)

        # print("trying")
        # print(abs(m_A.motor_data.multiturn_position - p_A))
        # print(abs(m_D.motor_data.multiturn_position - p_D))

        if (m_A.motor_data.speed == 0 and m_D.motor_data.speed == 0 and
                abs(m_A.motor_data.multiturn_position - p_A) < 0.1  and
                abs(m_D.motor_data.multiturn_position - p_D) < 0.1):
            break


# noinspection PyUnresolvedReferences
def load_cycle(file_n):
    mat_file = scipy.io.loadmat(f'cycles/cycle_{file_n}.mat')

    qA = mat_file['qA'][0]
    qD = mat_file['qD'][0]

    dqA = mat_file['dqA'][0]
    dqD = mat_file['dqD'][0]

    print(qA)
    print(qD)

    qA = -qA * 5
    qD = -qD * 5

    qA = qA + A_OFFSET
    qD = qD + D_OFFSET

    print(qA)
    print(qD)

    dqA = -dqA * 5
    dqD = -dqD * 5

    l = len(qA)
    print(l)

    if l != len(qD):
        print("ERROR: qA and qD are not the same size")
        exit(2)

    return qA, qD, dqA, dqD, l


def get_control_mode():
    if control_modes[control_index] == "position":
        return position_control

    elif control_modes[control_index] == "speed":
        return speed_control

    elif control_modes[control_index] == "shape":
        return shape_control

    return no_control()


def position_control():
    m_A.set_control_mode("position", qA[t])
    m_D.set_control_mode("position", qD[t])
    m_A.control()
    m_D.control()


def speed_control():
    m_A.set_control_mode("speed", dqA[t])
    m_D.set_control_mode("speed", dqD[t])
    m_A.control()
    m_D.control()


def shape_control():
    global t

    m_A.read_multiturn_once()
    current_positionA = m_A.motor_data.multiturn_position

    start = t - int(RANGE / 2)
    print(start)

    distances = []
    distances_indexes = []

    for i in range(RANGE):
        index = start + i

        if index < 0:
            index = l + index

        elif index >= l:
            index -= l

        print(index)
        print(abs(current_positionA - qA[index]))
        distances.append(abs(current_positionA - qA[index]))
        distances_indexes.append(index)

    sorted_lists = sorted(zip(distances, distances_indexes))
    distances, distances_indexes = zip(*sorted_lists)

    closest = float('inf')
    closest_index = 0

    m_A.read_multiturn_once()
    current_positionD = m_D.motor_data.multiturn_position

    for i in range(TOP_N):
        index = distances_indexes[i]

        if abs(current_positionD - qD[index]) < closest:
            closest_index = index

    t = closest_index
    print(f'This is t: {t}')

    m_A.set_control_mode("speed", dqA[t])
    m_D.set_control_mode("speed", dqD[t])

    m_A.control()
    m_D.control()


def fake_main():
    global changed_control_mode, changed_file_n, restart
    while True:
        print(f"File_n is {file_n}")
        print(f"Control is {control_modes[control_index]}")
        time.sleep(0.5)

        if changed_file_n:
            print("___________________________FILE_N")
            with lock:
                changed_file_n = False

        if changed_control_mode:
            print("____________________________CONTROL")
            with lock:
                changed_control_mode = False

        if restart:
            print("restart")
            restart = False


if __name__ == "__main__":

    file_n, control_index, debug, it = parse_arguments()
    qA, qD, dqA, dqD, l = load_cycle(file_n)

    log = datadump if debug else noop

    controller = find_controller()
    controller_thread = threading.Thread(target=read_controller_inputs, args=(controller,), daemon=True)
    controller_thread.start()

    # fake_main()

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_A = CanMotor(can0, MAX_SPEED=500, motor_id=7, gear_ratio=1, name="A")  # m_A
    m_D = CanMotor(can0, MAX_SPEED=500, motor_id=0, gear_ratio=1, name="D")  # m_D
    motors = [m_A, m_D]

    motor_listener = MotorListener(motor_list=motors)
    notifier = can.Notifier(can0, [motor_listener])

    restart_motors()

    time.sleep(1)
    input("Hey! I'm walking here!")

    set_initial_position(qA[0], qD[0])
    control = get_control_mode()

    t = 0
    i = 0

    try:
        while it == 0 or i < it:
            t += 1

            log()

            control()
            time.sleep(0.01)

            if t == l - 1:
                t = 0
                i += 1

            if restart:
                print("Restarting...")
                restart_motors()
                set_initial_position(qA[0], qD[0])

                t = 0
                with lock:
                    restart = False

            if changed_file_n:
                print("Changing .mat file...")
                load_cycle(file_n)
                set_initial_position(qA[0], qD[0])

                t = 0
                with lock:
                    changed_file_n = False

            if changed_control_mode:
                print("Changing control mode...")
                restart_motors()
                set_initial_position(qA[0], qD[0])
                control = get_control_mode()

                t = 0
                with lock:
                    changed_control_mode = False

        end()

    except (OSError, can.CanOperationError) as e:
        print("No crashing allowed")
        time.sleep(0.05)

    except KeyboardInterrupt:
        end()
