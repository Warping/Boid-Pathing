import math
import pygame
import random

class Boid:
    def __init__(self, position, parameters):
        """
        Initialize a Boid instance.

        :param position: A tuple (x, y) representing the position of the boid.
        :param parameters: A dictionary containing boid parameters such as max_speed, min_speed, etc.
        """
        self.position = position
        self.velocity = (0.0, 0.0)  # Initial velocity
        self.acceleration = (0, 0) # Initial acceleration
        self.target_angle = random.uniform(0, 2 * math.pi)  # Random target angle
        self.angle = random.uniform(0, 2 * math.pi)  # Random initial angle
        self.angular_velocity = 0 # Initial angular velocity
        self.angular_acceleration = 0 # Initial angular acceleration
        self.max_speed = parameters.get("max_speed", 4) # Maximum speed
        self.min_speed = parameters.get("min_speed", 0.8) # Minimum speed
        self.max_acceleration = parameters.get("max_acceleration", 0.2) # Maximum acceleration
        self.max_angular_velocity = parameters.get("max_angular_velocity", 0.05) # Maximum angular velocity
        self.min_angular_velocity = parameters.get("min_angular_velocity", 0.0) # Minimum angular velocity
        self.max_angular_acceleration = parameters.get("max_angular_acceleration", 0.2) # Maximum angular acceleration
        self.render_color = parameters.get("render_color", (100, 100, 100)) # Render color
        
        self.leader = False # Flag to indicate if the boid is a leader
        self.damping = 0.99 # Damping factor for velocity
        self.scale = 1 # Scale factor for rendering
        self.collided = False # Flag to indicate if the boid has collided
        self.poi_position = None # Point of interest (POI) position
        self.poi_time = 0 # Time of POI
        self.relay_boid = None # Relay boid
        self.max_old_pois = 10 # Maximum number of old POIs to keep
        self.old_pois = [] # List to keep track of old POIs
        self.old_times = [] # List to keep track of old times
        self.counter = 0 # Counter to keep track of time

    def update(self, screen):
        """
        Update the boid's position and velocity based on its acceleration.
        """
        # Apply angular force based on the target
        self.apply_angular_force(self.max_angular_acceleration)
        self.update_velocity()
        self.update_position(screen)
        self.update_rotation()
        self.counter += 1
    
    def update_rotation(self):
        """
        Rotate the boid's velocity by a given angle.
        """
        # Normalize angles to the range [-pi, pi]
        angle_diff = (self.target_angle - self.angle + math.pi) % (2 * math.pi) - math.pi

        # Apply angular acceleration based on the angle difference
        if abs(angle_diff) > 0.03:  # Threshold to avoid jitter
            self.angular_acceleration = max(-self.max_angular_acceleration, 
                                            min(self.max_angular_acceleration, angle_diff * 0.1))
        else:
            self.angular_acceleration = 0

        # Update angular velocity and apply damping
        self.angular_velocity += self.angular_acceleration
        self.angular_velocity *= self.damping

        # Limit angular velocity
        self.angular_velocity = max(-self.max_angular_velocity, 
                                     min(self.max_angular_velocity, self.angular_velocity))

        # Update the angle
        self.angle += self.angular_velocity
        self.angle %= 2 * math.pi  # Keep angle in the range [0, 2π]

        # Update the velocity based on the new angle
        speed = math.sqrt(self.velocity[0]**2 + self.velocity[1]**2)
        self.velocity = (speed * math.cos(self.angle), speed * math.sin(self.angle))
        
    def update_position(self, screen : pygame.Surface):
        """
        Update the boid's position based on its velocity.
        """
        vx, vy = self.velocity
        x, y = self.position
        self.position = (x + vx, y + vy)
        # Wrap around the screen edges
        if self.position[0] < 0:
            self.position = (screen.get_width(), self.position[1])
        elif self.position[0] > screen.get_width():
            self.position = (0, self.position[1])
        if self.position[1] < 0:
            self.position = (self.position[0], screen.get_height())
        elif self.position[1] > screen.get_height():
            self.position = (self.position[0], 0)
        
    def update_velocity(self):
        """
        Update the boid's velocity based on its acceleration.
        """
        ax, ay = self.acceleration
        # Limit the acceleration
        if ax*ax + ay*ay > self.max_acceleration * self.max_acceleration:
            length = math.sqrt(ax*ax + ay*ay)
            ax = ax / length * self.max_acceleration
            ay = ay / length * self.max_acceleration
        vx, vy = self.velocity
        # Find the projection of the acceleration on the velocity
        a_dot_v = ax * vx + ay * vy
        v_dot_v = vx * vx + vy * vy
        if v_dot_v == 0:
            # Use the acceleration directly if the velocity is zero
            a_proj_v = (ax, ay)
        else:
            a_proj_v = (a_dot_v / v_dot_v * vx, a_dot_v / v_dot_v * vy)
        # Update the velocity
        self.velocity = (vx + a_proj_v[0], vy + a_proj_v[1])
        # Damp the velocity
        self.velocity = (self.velocity[0] * self.damping, self.velocity[1] * self.damping)
        # Limit the speed
        speed = math.sqrt(self.velocity[0]**2 + self.velocity[1]**2)
        if speed == 0:
            return
        if speed > self.max_speed:
            self.velocity = (self.velocity[0] / speed * self.max_speed, self.velocity[1] / speed * self.max_speed)
        elif speed < self.min_speed:
            self.velocity = (self.velocity[0] / speed * self.min_speed, self.velocity[1] / speed * self.min_speed)
        
    def apply_force(self, force):
        """
        Apply a force to the boid's acceleration.

        :param force: A tuple (fx, fy) representing the force to apply.
        """
        self.acceleration = (self.acceleration[0] + force[0], self.acceleration[1] + force[1])
        if force[0] != 0 or force[1] != 0:
            self.target_angle = math.atan2(self.acceleration[1], self.acceleration[0])
    
    def apply_angular_force(self, force):
        """
        Apply an angular force to the boid's angular acceleration.

        :param force: A float representing the angular force to apply.
        """
        self.max_angular_acceleration = force
            
    
    def reset_acceleration(self):
        """
        Reset the boid's acceleration to zero.
        """
        self.acceleration = (0, 0)
        self.angular_acceleration = 0
        
    def save_poi(self):
        """
        Save the point of interest (POI) for the boid.
        """
        if self.poi_position is not None:
            self.old_pois.append(self.poi_position)
            self.old_times.append(self.counter)
            if len(self.old_pois) > self.max_old_pois:
                self.old_pois.pop(0)
                self.old_times.pop(0)
    
    def render(self, screen):
        """
        Render the boid as a triangle on the screen.
        """
        # Calculate the points of the triangle based on the boid's position and angle
        x, y = self.position
        points = [
            (x + 10 * self.scale * math.cos(self.angle), y + 10 * self.scale * math.sin(self.angle)),
            (x + 5 * self.scale * math.cos(self.angle + 2 * math.pi / 3), y + 5 * self.scale * math.sin(self.angle + 2 * math.pi / 3)),
            (x + 5 * self.scale * math.cos(self.angle - 2 * math.pi / 3), y + 5 * self.scale * math.sin(self.angle - 2 * math.pi / 3))
        ]

        # Draw the triangle on the screen
        if self.render_color != (100, 100, 100):
            # Draw the triangle with a different color if the render color is set
            pygame.draw.polygon(screen, self.render_color, points)
        else:
            if self.poi_position is not None and self.relay_boid is self:
                # Draw the triangle with a different color if the point of interest is set
                pygame.draw.polygon(screen, (0, 255, 0), points)
            elif self.poi_position is not None and self.relay_boid is not None:
                # Draw the triangle with a different color if the point of interest is set
                pygame.draw.polygon(screen, (255, 255, 0), points)
            else:
                pygame.draw.polygon(screen, (100, 100, 100), points)

        # Render the velocity vector
        vx, vy = self.velocity
        pygame.draw.line(screen, (255, 128, 255), (x, y), (x + vx * 10 * self.scale, y + vy * 10 * self.scale), 2)
