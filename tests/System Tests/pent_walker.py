import sys
import os

from orca.punctuation_settings import infinity

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

control_modes = {
    "p": "position",
    "v": "speed",
    "s": "shape"
}

def no_function():
    print("No control mode")


def end():
    m_A.set_control_mode("position", A_OFFSET)
    m_D.set_control_mode("position", D_OFFSET)

    m_A.control()
    m_D.control()

    time.sleep(4)

    for motor in motors:
        motor.stop_all_tasks()
        motor.motor_off()

    notifier.stop()
    core.CANHelper.cleanup("can0")
    can0.shutdown()
    print("Exiting")
    exit(0)


def load_cycle(filename):
    mat_file = scipy.io.loadmat(f'cycles/{filename}')

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

def position_control():
    m_A.set_control_mode(control_mode, qA[t])
    m_D.set_control_mode(control_mode, qD[t])
    m_A.control()
    m_D.control()


def speed_control():
    m_A.set_control_mode(control_mode, dqA[t])
    m_D.set_control_mode(control_mode, dqD[t])
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


if __name__ == "__main__":

    if len(sys.argv) >= 3:
        filename = f'cycle_{sys.argv[1]}.mat'
        control_mode = control_modes[sys.argv[2]]
        print(control_mode)

    else:
        print("ERROR: Filename or control mode not provided")
        exit(1)

    if len(sys.argv) >= 4:
        it = int(sys.argv[2])

    else:
        it = 0

    qA, qD, dqA, dqD, l = load_cycle(filename)

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_A = CanMotor(can0, MAX_SPEED=500, motor_id=7, gear_ratio=1, name="A")  # m_A
    m_D = CanMotor(can0, MAX_SPEED=500, motor_id=0, gear_ratio=1, name="D")  # m_D
    motors = [m_A, m_D]

    motor_listener = MotorListener(motor_list=motors)
    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.write_acceleration(0x00, np.uint32(60000))
        motor.write_acceleration(0x01, np.uint32(60000))

        motor.write_acceleration(0x02, np.uint32(100))
        motor.write_acceleration(0x03, np.uint32(100))

        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")

    m_A.set_control_mode("position", qA[0])
    m_D.set_control_mode("position", qD[0])

    print(qA[0])
    print(qD[0])

    m_A.control()
    m_D.control()

    time.sleep(5)

    t = 0
    i = 0

    motionA = qA if control_mode == "position" else dqA
    motionD = qD if control_mode == "position" else dqD

    control = no_function

    if control_mode == "position":
        control = position_control

    if control_mode == "speed":
        control = speed_control

    if control_mode == "shape":
        control = shape_control

    try:
        while it == 0 or i < it:
            # for motor in motors:
            #     motor.read_status_once()
            #     time.sleep(0.02)
            #     motor.read_multiturn_once()
            #     time.sleep(0.02)
            #     motor.read_motor_state_once()
            #     time.sleep(0.02)
            #
            # for motor in motors:
            #     motor.datadump()
            #     time.sleep(0.02)

            t += 1
            control()
            time.sleep(0.01)

            if t == l - 1:
                t = 0
                i += 1

        end()

    except KeyboardInterrupt:
        end()
