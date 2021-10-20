import numpy as np

# when leds are off it means they're on the white line
led1 = 1
led2 = 0
led3 = 0
led4 = 1


def follow_line(led1, led2, led3, led4):

    while True:
        # add something to check if the previous command is the same as the current one
        # if the alignment of the leds is correct, all of them are on the black surface and high
        if led1 == 1 and led2 == 1 and led3 == 1 and led4 == 1:
            print("go straight")
            return

        # if all the leds are on white because of the horizontal line
        elif led1 == 0 and led2 == 0 and led3 == 0 and led4 == 0:
            print("go straight") # do not return in this case bc the path could still be wrong

        # if the robot is slightly tilted to the left
        elif led1 == 1 and led2 == 1 and led3 == 0 and led4 == 0:
            while led3 == 0:
                print("go right, right motor speed to half, left motor full speed")

        # if the robot is slightly tilted to the right
        elif led1 == 1 and led2 == 0 and led3 == 1 and led4 == 1:
            while led2 == 0:
                print("go left, left motor speed to half, right motor full speed")

        # if either two pairs are on and the other two are off
        elif (led1 == 0 and led2 == 0 and led3 == 1 and led4 == 1) or (led3 == 0 and led4==0 and led1 == 1 and led2 == 1)
            print("turn 90 degrees around")

        else:
            print("turn 90 degrees around")











