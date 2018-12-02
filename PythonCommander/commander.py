#!/usr/bin/env python
import math
import time
import numpy as np
from py4j.java_gateway import JavaGateway

from forward_inverse_kinematics import inv_kin
from robot_configuration import L
from trajectory_planning import plan
from flask import Flask, request
import threading

pos = [100, 0, 100]
handrot = 0
scriptRunning = False

app = Flask(__name__, static_url_path='')

movement = {
    'Forward': False,
    'Backward': False,
    'Left': False,
    'Right': False,
    'Up': False,
    'Down': False,
    'Open grip': False,
    'Close grip': False,
    'Handrot left': False,
    'Handrot right': False,
}

@app.route('/')
def root():
    return app.send_static_file('index.html')


@app.route('/move', methods=['POST'])
def move():
    cmd = request.get_json()['cmd']
    movement[cmd] = True
    return 'ok'


@app.route('/stop', methods=['POST'])
def stop():
    cmd = request.get_json()['cmd']
    movement[cmd] = False
    return 'ok'


@app.route('/return', methods=['GET'])
def returnback():
    global pos, handrot
    pos = [100, 0, 100]
    handrot = 0
    return 'ok'


@app.route('/script1', methods=['GET'])
def script1():
    global scriptRunning
    scriptRunning = True
    print("eeee")
    return 'ok'


def remoteControl():
    print("Start remote control")

    global pos, handrot, scriptRunning

    gateway = JavaGateway()
    robotArmController = gateway.entry_point

    step = 2

    while True:
            if scriptRunning:
                pos_plan = plan()
                robotArmController.setJointAngles(pos_plan[0, 0], pos_plan[1, 0], pos_plan[2, 0])
                robotArmController.sendData()
                time.sleep(0.5)
                for i in range(pos_plan.shape[1]):
                    q = pos_plan[:, i]
                    print(q)
                    robotArmController.setJointAngles(q[0], q[1], q[2])
                    robotArmController.sendData()
                    time.sleep(.01)
                scriptRunning = False
                continue
            pos_prev = pos.copy()
            if movement['Forward']:
                pos[0] += step
            elif movement['Backward']:
                pos[0] -= step
            elif movement['Left']:
                pos[1] += step
            elif movement['Right']:
                pos[1] -= step
            elif movement['Up']:
                pos[2] += step
            elif movement['Down']:
                pos[2] -= step
            elif movement['Handrot left']:
                handrot -= step
            elif movement['Handrot right']:
                handrot += step


            q = inv_kin(pos, L)
            if math.isnan(pos[0]) or math.isnan(pos[1]) or math.isnan(pos[2]):
                pos = pos_prev
                q = inv_kin(pos, L)
            # print (q)
            robotArmController.setJointAngles(q[0], q[1], q[2])
            robotArmController.setHandrot(handrot)

            if movement['Open grip']:
                robotArmController.setGrip(True, False)
            elif movement['Close grip']:
                robotArmController.setGrip(True, True)
            else:
                robotArmController.setGrip(False, True)

            robotArmController.sendData()
            time.sleep(0.01)


def flaskStart():
    app.run(host='0.0.0.0')


if __name__ == "__main__":
    threading.Thread(target=flaskStart).start()
    remoteControl()

# def demonstrate(pos):
#     gateway = JavaGateway()
#     robotArmController = gateway.entry_point
#     while True:
#         for i in range(pos.shape[1]):
#             q = pos[:, i]
#             print(q)
#             robotArmController.setJointAngles(q[0], q[1], q[2])
#             robotArmController.sendData()
#             time.sleep(.01)
#
# # demonstrate()
# pos = plan()
# demonstrate(pos)

# q = inv_kin(np.array([150, 30, -20]), L)
# print(q)
# x = forw_kin(q, L)
# print(x)

# print(inv_kin(150, 150, 110))

# test()


# def talker():
#     pub = rospy.Publisher('/scara/joint1_position_controller/command', Float64, queue_size=10)
#     pub2 = rospy.Publisher('/scara/joint2_position_controller/command', Float64, queue_size=10)
#     pub3 = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(3)  # 10hz
#
#     ############Test it!##################
#     # set of desired (x,y) hand positions
#     z = 0.6
#     r = 0.3
#     center_x = 1.2
#     center_y = 1.2
#
#     a = np.arange(0, 2*math.pi, 0.5)
#
#     while not rospy.is_shutdown():
#         for ac in range(len(a)):
#                 x = center_x + r * np.cos(ac)
#                 y = center_y + r * np.sin(ac)
#                 # run the inv_kin function, get the optimal joint angles
#               #  rospy.loginfo("%f %f %f" % (x,y,z))
#
#                 q = inv_kin(x, y, z)
#                 # find the (x,y) position of the hand given these angles
#                 rospy.loginfo(q)
#                 pub.publish(q[0])
#                 pub2.publish(q[2])
#                 pub3.publish(q[2])
#                 rate.sleep()
#
#
# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
