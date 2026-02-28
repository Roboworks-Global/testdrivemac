# APP_NAME: ObstacleRotateAndMove
import time
import testdrive as td

def main():
    robot = td.Robot()
    
    # Continuously check for obstacles
    while True:
        distance = robot.get_distance()  # in meters
        
        if distance < 0.5:  # obstacle detected within 0.5m
            print(f"Obstacle detected at {distance:.2f}m!")
            
            # Rotate according to current rotation parameter
            rotation_angle = robot.get_parameter("rotation_angle")
            print(f"Rotating by {rotation_angle} degrees...")
            robot.rotate(rotation_angle)
            
            # Wait for rotation to complete
            time.sleep(1)
            
            # Move forward 0.5 meters and stop
            print("Moving forward 0.5 meters...")
            robot.move_forward(0.5)
            
            # Wait for movement to complete
            time.sleep(2)
            
            print("Done. Resuming obstacle detection...")
        
        time.sleep(0.1)  # polling interval

if __name__ == "__main__":
    main()
