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
    SLEEP -= 0.1
    HUNGER -= 0.2
    THIRST -= 0.2
    #print("S:", SLEEP, ", H:", HUNGER, ", T:", THIRST)  # DEBUG
    
def updateSensors(object):
    global SLEEP, HUNGER, THIRST
    red = 0
    green = 0
    blue = 0
    
    # Determine the object based on RGB colours detected
    red, green, blue = object.get_camera_image(5)
    
    # Decrement SLEEP, HUNGER, THIRST variables
    meta()
    
def foundFood(object):
    if (object.red >= 23 and object.red <= 42 ) and (object.green >= 42 and object.green <= 80) and (object.blue >= 26 and object.blue <= 51):
        return True
    else:
        return False

def foundWater(object):
    if (object.red >= 12 and object.red <= 17 ) and (object.green >= 45 and object.green <= 62) and (object.blue >= 48 and object.blue <= 76):
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
            
            """if foundFood(robot1):
                print("i see food")
            if foundWater(robot1):
                print("i see water")"""

            
            #robot1.move_forward()
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
