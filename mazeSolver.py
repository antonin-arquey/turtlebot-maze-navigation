#!/usr/bin/env python

from Turtlebot import Turtlebot
import socket
import rospy
import sys
import argparse

# Map is created here
# mazeMap contains the information where the robot needs to turn
# turns contains the information of the turn themselves

### Example
# mazeMap = []
# mazeMap.append(['Right', 1])
# mazeMap.append(['Left', 2])

# turns = []
# turns.append([90, 'Right'])
# turns.append([90, 'Left'])

# Here we have :
# - At the first hole on the right (mazeMap), turn 90 degrees on the right (turns)
# - Then at the second hole on the left, turn 90 degrees on the left



mazeMap = []

mazeMap.append(['Right', 1])
mazeMap.append(['Left', 2])
mazeMap.append(['Left', 1])

turns = []

turns.append([90, 'Right'])
turns.append([90, 'Left'])
turns.append([90, 'Left'])

def solveMaze(name=""):
    bot = Turtlebot(name)
    hasTurned = False

    while not rospy.is_shutdown():
        rospy.loginfo("Next Detection : " + mazeMap[0][0] + " in " + str(mazeMap[0][1]))
        if mazeMap[0][0] == 'Left': # Next way is on the left
            if bot.detectHoleLeft() and not hasTurned:
                if mazeMap[0][1] <= 1: # If we turn there
                    bot.stop()
                    if turns[0][1] == 'Left':
                        bot.turnLeft(turns[0][0])
                    else:
                        bot.turnRight(turns[0][0])
                    turns.pop(0)
                    mazeMap.pop(0)
                    hasTurned = True
                else:
                    mazeMap[0][1] = mazeMap[0][1] - 1 # we are closing to the next turn
                    while bot.detectHoleLeft(): # wait that this turn is passed
                        pass
            else:
                bot.moveForward(0.1)
                while (bot.detectHoleLeft()) and hasTurned:
                    rospy.loginfo("Waiting end of hole after turn")
                hasTurned = False

        else: # Nex way is on the right
            if bot.detectHoleRight() and not hasTurned:
                if mazeMap[0][1] <= 1:
                    bot.stop()
                    if turns[0][1] == 'Left':
                        bot.turnLeft(turns[0][0])
                    else:
                        bot.turnRight(turns[0][0])
                    turns.pop(0)
                    mazeMap.pop(0)
                    hasTurned = True
                else:
                    mazeMap[0][1] = mazeMap[0][1] - 1
                    while bot.detectHoleRight():
                        pass
            else:
                bot.moveForward(0.1)
                while (bot.detectHoleRight()) and hasTurned:
                    rospy.loginfo("Waiting end of hole after turn")
                hasTurned = False


def run(name):
    rospy.loginfo('Starting Bot ...')
    rospy.init_node('turtlebot3_maze')
    solveMaze(name)

def getData(ip, port):
    TCP_IP = ip
    TCP_PORT = int(port)
    BUFFER_SIZE = 1024
    MESSAGE = 'TurtleBot'

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE.encode())
    data = s.recv(BUFFER_SIZE)
    s.close()
    turns = []
    mazeMap = []
    for action in data.decode().split(';'):
        if action:
            action = action.split(' ')
            nieme = int(action[0])
            right = int(action[1])
            mazeMap.append(['Right' if right else 'Left', nieme])
            turns.append([90, 'Right' if right else 'Left'])

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', help="namespace of the bot")
    parser.add_argument('-a', '--address', help="address of the server")
    parser.add_argument('-p', '--port', help="port of the server")
    args = parser.parse_args()
    if args.address and args.port:
        getData(args.address, args.port)
        print(mazeMap)
        print(turns)
    run(args.name)
