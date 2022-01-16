from picarx_improved import Picarx
import time

if __name__ == "__main__":

    px = Picarx()
    dir_angle = 0

    # Drive forward in a straight line
    def drive_forward(dir_angle):
        px.set_dir_servo_angle(dir_angle)
        time.sleep(0.5)
        px.forward(50)
        time.sleep(3)
        px.stop()

    # Drive backward in a straight line
    def drive_backward(dir_angle):
        px.set_dir_servo_angle(dir_angle)
        time.sleep(0.5)
        px.backward(50)
        time.sleep(3)
        px.stop()

    # Parallel park
    def parallel_park():
        px.forward(50)
        time.sleep(0.5)
        for angle in range(0, dir*35, dir*1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(dir*35, -dir*35, -dir*1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(-dir*35, 0, dir*1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        time.sleep(0.5)
        px.stop()
        time.sleep(0.5)
        px.backward(50)
        time.sleep(0.5)
        px.stop()

    # Perform a K turn
    def k_turn():
        px.forward(50)
        time.sleep(1)
        for angle in range(0, dir*35, dir*1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(dir*35, 0, -dir*1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        time.sleep(0.5)
        px.stop()
        time.sleep(0.5)
        px.backward(50)
        time.sleep(1)
        for angle in range(0, -dir * 35, -dir * 1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(-dir*35, 0, dir*1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        px.stop()
        time.sleep(0.5)
        px.set_dir_servo_angle(0)
        px.forward(50)
        time.sleep(1)
        for angle in range(0, dir * 35, dir * 1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(dir*35, 0, -dir * 1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        time.sleep(1)
        px.stop()

    while True:
        print("Please choose a maneuver:"
              " w=forward"
              " s=backward"
              " a=parallel park"
              " d=K-turn"
              " q=quit")

        maneuver = input()

        if maneuver == "w":
            print("Choose an angle [-35 to 35]")
            dir_angle = int(input())
            drive_forward(dir_angle)
        elif maneuver == "s":
            print("Choose an angle [-35 to 35]")
            dir_angle = int(input())
            drive_backward(dir_angle)
        elif maneuver == "q":
            break
        else:
            print("Please choose a direction:"
                  " l=left"
                  " r=right"
                  " q=quit")
            direction = input()

            if direction == "l":
                dir = -1
            elif direction == "r":
                dir = 1
            elif direction == "q":
                break
            else:
                print("Direction not chosen")

            if maneuver == "a":
                parallel_park()
            elif maneuver == "d":
                k_turn()
            else:
                print("Maneuver not chosen, please try again")

        time.sleep(8)
