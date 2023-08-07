import robot, time, random 

# Parameters
HUNGER = 1000
THIRST = 1000
SLEEP = 1000

# Variables
hunger_threshold = 500
thirst_threshold = 500
sleep_threshold = 500


# Functions
def eating():
    global HUNGER
    if HUNGER < 1000:
        HUNGER += 30

def sleeping():
    global SLEEP   
    if SLEEP < 1000:
        SLEEP += 10
     
def drinking():
    global THIRST
    if THIRST < 1000:
        THIRST += 50
     
def alive():
    if SLEEP <= 0:
        return False
    elif HUNGER <= 0:
        return False
    elif THIRST <= 0:
        return False
    else:
        return True
        
def meta():
    global SLEEP, HUNGER, THIRST
    SLEEP -= 1
    HUNGER -= 0.2
    THIRST -= 0.2
    #print("S:", SLEEP, ", H:", HUNGER, ", T:", THIRST)  # DEBUG

# Randomised multiplier to move robot forward depending on distance to obstacles
def Behaviour0(object):
    value = random.uniform(0.3, 0.5)
    object.move(value, value)

# Avoid obstacles by turning left--
def Behaviour1(object):
    if object.front_obstacles_detected():
        object.move_backwards()
        object.turn_left()
    elif object.right_obstacles_detected():
        object.turn_left()
    elif object.left_obstacles_detected():
        object.turn_right()
    else:
        object.move_forward()
        
# Move robot turn left 180 degrees
def Behaviour3(object):
    object.move_forward()
    object.turn_left()
    object.turn_left()
    object.turn_left()

def updateSensors(object):
    global SLEEP, HUNGER, THIRST
    red = 0
    green = 0
    blue = 0
    
    # Determine the object based on RGB colours detected
    red, green, blue = object.get_camera_image(5)
    
    """if (red >= 108 and red <= 143 ) and (green >= 162 and green <= 170) and (blue >= 131 and blue <= 140):
        print("i see food")"""
        
    """if (red >= 70 and red <= 129 ) and (green >= 135 and green <= 161) and (blue >= 125 and blue <= 167):
        print("i see water")"""
    
    # Decrement SLEEP, HUNGER, THIRST variables
    meta()
    
def foundFood(object):
    if (object.red >= 90 and object.red <= 112 ) and (object.green >= 147 and object.green <= 158) and (object.blue >= 125 and object.blue <= 129):
        return True
    else:
        return False

def foundWater(object):
    if (object.red >= 77 and object.red <= 97 ) and (object.green >= 139 and object.green <= 143) and (object.blue >= 139 and object.blue <= 162):
        return True
    else:
        return False



def main():
    range = 0.0
    robot1 = robot.ARAP()
    robot1.init_devices()

    while True and alive() != False: # Runs forever
        try:
            robot1.reset_actuator_values()
            range = robot1.get_sensor_input()
            #print("Distance range =", range)  # DEBUG
            robot1.blink_leds()
            
            updateSensors(robot1)
            #Behaviour1(robot1)
            
            """if foundFood(robot1):
                print("i see food")
                #Behaviour food (robot1)
            elif foundWater(robot1):
                print("i see water")
                #Behaviour water (robot1)"""
            
            robot1.move_forward()
            #robot1.run_braitenberg()
            #robot1.set_actuators()
            robot1.step()
        except KeyboardInterrupt:  # Will break out of while loop if Ctrl+C pressed.
            break
        finally:
            if not alive():
                print("\nRobot is dead\n")

if __name__ == "__main__":
    main()
