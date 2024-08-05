from robomaster import robot
from robomaster import armor
import time

speed = 0.7
hit = False

def hit_callback(sub_info, ep_robot):
    hit = True
    armor_id, hit_type = sub_info
    print("hit event: hit_comp:{0}, hit_type:{1}".format(armor_id, hit_type))
    ep_robot.chassis.stop()
    if armor_id==1:
        ep_robot.chassis.move(x=0.5, y=0, z=90.0, xy_speed=speed).wait_for_completed()
    else:
        ep_robot.chassis.move(x=-0.5, y=0, z=-90.0, xy_speed=speed).wait_for_completed()
    hit = False

def main():
    # Verbindung mit Robomaster herstellen
    ep_robot = robot.Robot()
    ep_robot.initialize()
    
    ep_armor = ep_robot.armor

    ep_armor.set_hit_sensitivity(comp="all", sensitivity=10)

    ep_armor.sub_hit_event(hit_callback, ep_robot)

    try:
        while True:
            if hit==False:
                ep_robot.chassis.move(x=0.5, y=0, z=0, xy_speed=speed*2).wait_for_completed()

            #obstacle_info = robomaster.chassis.front_distance_detector
            #if obstacle_info.is_obstacle_detected:
            #    print("Hindernis erkannt. Stoppe den Roboter.")
            #    robomaster.chassis.stop()
            #    break

            time.sleep(0.1)

    finally:
        time.sleep(15)
        ep_armor.unsub_hit_event()
        ep_robot.chassis.stop()
        ep_robot.close()


if __name__ == "__main__":
    main()
