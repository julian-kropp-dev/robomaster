import robomaster
from robomaster import robot
import termios, fcntl, sys, os
import time


def forward(seconds):
    print("Moving forward for {0} seconds.".format(seconds))
    ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.7).wait_for_completed()

def backward(seconds):
    print("Moving backward for {0} seconds.".format(seconds))
    ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.7).wait_for_completed()
    # Code hier einfügen, damit der RM rückwärts fährt


def left(seconds):
    print("Moving left for {0} seconds.".format(seconds))
    ep_chassis.move(x=0.0, y=-0.5, z=0, xy_speed=0.7).wait_for_completed()
    # Code hier einfügen, damit der RM seitwärts links fährt


def right(seconds):
    print("Moving right for {0} seconds.".format(seconds))
    ep_chassis.move(x=0.0, y=0.5, z=0, xy_speed=0.7).wait_for_completed()
    # Code hier einfügen, damit der RM seitwärts rechts fährt
    
def turnLeft(seconds):
    print("Turning left for {0} seconds.".format(seconds))
    ep_chassis.move(x=0.0, y=0, z=10, xy_speed=0.7).wait_for_completed()
    # Code hier einfügen, damit der RM sich nach links dreht


def turnRight(seconds):
    print("Turning right for {0} seconds.".format(seconds))
    ep_chassis.move(x=0.0, y=0, z=-10, xy_speed=0.7).wait_for_completed()
    # Code hier einfügen, damit der RM sich nach rechts dreht


if __name__ == '__main__':
    # initialize robomaster
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='ap')
    ep_chassis = ep_robot.chassis

    version = ep_robot.get_version()
    print("Robot version: {0}".format(version))
    
    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    try:
        while True:
            try:
                c = sys.stdin.read(1)
                if c:
                    if c == 'w':
                	    forward(1)
                    elif c == 's':
                        backward(1)
                    elif c == 'a':
    	                left(1)
                    elif c == 'd':
    	                right(1)
                    elif c == 'q':
    	                turnLeft(1)
                    elif c == 'e':
    	                turnRight(1)
                    time.sleep(0.5)
            except IOError: pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

        ep_robot.close()
