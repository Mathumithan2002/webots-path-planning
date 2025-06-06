from controller import Robot
import numpy as np

# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize the camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Maximum speed of the e-puck motors
MAX_SPEED = 6.28  # rad/s (approx. 1000 steps/s for e-puck)

# Camera properties (e-puck camera is 52x39 pixels by default in Webots)
width = camera.getWidth()
height = camera.getHeight()

# Main loop
while robot.step(timestep) != -1:
    # Get the camera image
    image = camera.getImage()
    
    # Convert the image to a numpy array for processing
    # Webots camera image is in BGRA format (4 bytes per pixel: Blue, Green, Red, Alpha)
    image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))

    # Extract the red channel (or grayscale by averaging RGB) to simplify processing
    # We'll use a grayscale approximation: (R + G + B) / 3
    grayscale = image_array[:, :, 0:3].mean(axis=2).astype(np.uint8)

    # Divide the image into three regions: left, center, right
    # We'll check the bottom part of the image to detect nearby obstacles
    bottom_part = grayscale[height-10:height, :]  # Last 10 rows
    left_region = bottom_part[:, 0:width//3].mean()  # Left third
    center_region = bottom_part[:, width//3:2*width//3].mean()  # Center third
    right_region = bottom_part[:, 2*width//3:width].mean()  # Right third

    # Threshold to detect obstacles (adjust based on your maze colors)
    # In the image, walls are black (low brightness), free space is white (high brightness)
    threshold = 200 # Adjust this based on testing (0-255 scale)

    # Determine if there are obstacles
    front_obstacle = center_region < threshold
    left_obstacle = left_region < threshold
    right_obstacle = right_region < threshold

    # Default speeds
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED

    # Adjust speeds based on camera input
    if front_obstacle:
        # Turn right if obstacle in front
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    elif left_obstacle:
        # Turn right if obstacle on left
        left_speed = 0.5 * MAX_SPEED
        right_speed = 0.2 * MAX_SPEED
    elif right_obstacle:
        # Turn left if obstacle on right
        left_speed = 0.2 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

    # Apply speeds to motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed) 