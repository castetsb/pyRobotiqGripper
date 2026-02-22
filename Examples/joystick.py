import pygame
from pyrobotiqgripper import RobotiqGripper

def map_0_255(x):
    """
    Map a value from [-1, 1] to [0, 255].
    """
    return int((x + 1) * (255-0)/(1-(-1))+0)

keep_running = True

pygame.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
js.init()
print("Joystick:", js.get_name())

gripper = RobotiqGripper("COM8")
gripper.resetActivate()

while keep_running:
    try:
        pygame.event.pump()
        joy0 = js.get_axis(3)  # Right stick Y
        pos = map_0_255(joy0)
        gripper.realTimeMove(pos)

    except KeyboardInterrupt:
        keep_running = False
