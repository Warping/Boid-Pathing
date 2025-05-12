import math
import random
import pygame
from boid import Boid
import matplotlib.pyplot as plt
import numpy as np


class Boids:
    def __init__(self, 
                 screen : pygame.Surface, 
                 boid_count, 
                 align_factor, 
                 cohesion_factor, 
                 separation_factor, 
                 targetting_factor, 
                 max_distance=60,
                 protection_distance=35,
                 collision_distance=5,
                 render_model=True, 
                 boid_parameters=None):
        """
        Initialize a Boids instance.

        :param screen: The pygame screen surface to render the boids on.
        :param boid_count: The number of boids to create.
        :param align_factor: The factor for alignment behavior.
        :param cohesion_factor: The factor for cohesion behavior.
        :param separation_factor: The factor for separation behavior.
        :param targetting_factor: The factor for targetting behavior.
        :param max_distance: The maximum distance for neighbor interactions.
        :param protection_distance: The distance for protection behavior.
        :param collision_distance: The distance for collision detection.
        :param render_model: Whether to render the model interactions or not.
        :param boid_parameters: A dictionary containing boid parameters such as max_speed, min_speed, etc.
        """
        # Initialize the boids with random positions and parameters
        if boid_parameters is None:
            boid_parameters = {
                "max_speed": 4,
                "min_speed": 0.8,
                "max_acceleration": 0.2,
                "max_angular_velocity": 0.05,
                "min_angular_velocity": 0.0,
                "max_angular_acceleration": 0.2
            }
        self.boid_array = [Boid((random.randint(0, screen.get_width()), random.randint(0, screen.get_height())), boid_parameters) for _ in range(boid_count)]
        self.boid_array[0].leader = True
        self.align_factor = align_factor # Factor for alignment behavior
        self.cohesion_factor = cohesion_factor # Factor for cohesion behavior
        self.separation_factor = separation_factor # Factor for separation behavior
        self.targetting_factor = targetting_factor # Factor for targetting behavior
        self.collision_counts = [] # List to store the number of collisions
        self.target_positions = [] # List to store the target positions
        self.avg_distances = [] # List to store the average distances to the target
        self.found_poi_counts = [] # List to store the number of boids that found the target POI
        self.avg_poi_positions = [] # List to store the average distances to the POI
        self.max_distance = max_distance # Maximum distance for neighbor interactions
        self.protection_distance = protection_distance # Distance for protection behavior
        self.collision_distance = collision_distance # Distance for collision detection
        self.screen = screen # The pygame screen surface to render the boids on
        self.remove_boid = False # Whether to remove the boid on collision
        self.render_model = render_model # Whether to render the model interactions or not
    
    def update(self, target):
        """
        Update the boids' positions and velocities based on their interactions.
        """
        boid : Boid
        # Shuffle the boid array to avoid order bias
        random.shuffle(self.boid_array)
        for boid in self.boid_array:
            boid.collided = False
            # Calculate the forces acting on the boid
            alignment_force = self.calculate_alignment(boid)
            cohesion_force = self.calculate_cohesion(boid)
            separation_force = self.calculate_separation(boid)
            targetting_force = self.calculate_targetting(boid, target)
            
            # Apply the forces to the boid
            boid.apply_force(alignment_force)
            boid.apply_force(cohesion_force)
            boid.apply_force(separation_force)
            boid.apply_force(targetting_force)
            
        # Relay the targetting info to the other boids
        for boid in self.boid_array:
            for other_boid in self.boid_array:
                # Check if 2 boids are too close to each other
                if boid != other_boid:
                    distance = math.sqrt((other_boid.position[0] - boid.position[0])**2 + (other_boid.position[1] - boid.position[1])**2)
                    if distance < self.collision_distance:
                        boid.collided = True
                        other_boid.collided = True
                if boid.poi_position != other_boid.poi_position:
                    distance_eachother = math.sqrt((other_boid.position[0] - boid.position[0])**2 + (other_boid.position[1] - boid.position[1])**2)
                    if distance_eachother < self.max_distance:
                        other_boid.poi_position = boid.poi_position
                        other_boid.relay_boid = boid
                        other_boid.old_pois = boid.old_pois
                        if boid.poi_time > other_boid.poi_time:
                            other_boid.poi_time = boid.poi_time
                        else:
                            boid.poi_time = other_boid.poi_time
            boid.update(self.screen)
        # Count the number of collisions for each boid
        count = 0
        for boid in self.boid_array:
            if boid.collided:
                if self.remove_boid:
                    # Remove the boid from the array
                    self.boid_array.remove(boid)
                count += 1
        self.collision_counts.append(count)
        # Calculate the average distance between a boid and the target
        total_distances = []
        for boid in self.boid_array:
            distance = math.sqrt((boid.position[0] - target[0])**2 + (boid.position[1] - target[1])**2)
            total_distances.append(distance)
        avg_distance = sum(total_distances) / len(total_distances)
        self.avg_distances.append(avg_distance)
        # Calculate the average position of the pois
        avg_poi_position = (0, 0)
        count = 0
        for boid in self.boid_array:
            if boid.poi_position is not None:
                avg_poi_position = (avg_poi_position[0] + boid.poi_position[0], avg_poi_position[1] + boid.poi_position[1])
                count += 1
        if count > 0:
            avg_poi_position = (avg_poi_position[0] / count, avg_poi_position[1] / count)
            self.avg_poi_positions.append(avg_poi_position)
        else:
            self.avg_poi_positions.append((None, None))
        # Count the number of boids that found a point of interest (POI)
        found_poi_count = 0
        for boid in self.boid_array:
            if boid.poi_position == target:
                found_poi_count += 1
        self.found_poi_counts.append(found_poi_count)
        
        # Append the target position to the list
        self.target_positions.append(target)
                
        
    def calculate_alignment(self, boid):
        """
        Calculate the alignment force for a boid based on its neighbors.

        :param boid: The Boid instance to calculate the alignment force for.
        :return: A tuple (fx, fy) representing the alignment force.
        """
        avg_velocity = (0, 0)
        count = 0
        for other_boid in self.boid_array:
            if other_boid != boid:
                distance = math.sqrt((other_boid.position[0] - boid.position[0])**2 + (other_boid.position[1] - boid.position[1])**2)
                if distance < self.max_distance:
                    avg_velocity = (avg_velocity[0] + other_boid.velocity[0], avg_velocity[1] + other_boid.velocity[1])
                    count += 1
        if count > 0:
            avg_velocity = (avg_velocity[0] / count, avg_velocity[1] / count)
            return (self.align_factor * (avg_velocity[0] - boid.velocity[0]), self.align_factor * (avg_velocity[1] - boid.velocity[1]))
        return (0, 0)
    
    def calculate_cohesion(self, boid):
        """
        Calculate the cohesion force for a boid based on its neighbors.

        :param boid: The Boid instance to calculate the cohesion force for.
        :return: A tuple (fx, fy) representing the cohesion force.
        """
        center_of_mass = (0, 0)
        count = 0
        for other_boid in self.boid_array:
            if other_boid != boid:
                distance = math.sqrt((other_boid.position[0] - boid.position[0])**2 + (other_boid.position[1] - boid.position[1])**2)
                if distance < self.max_distance:
                    center_of_mass = (center_of_mass[0] + other_boid.position[0], center_of_mass[1] + other_boid.position[1])
                    count += 1
        if count > 0:
            center_of_mass = (center_of_mass[0] / count, center_of_mass[1] / count)
            return (self.cohesion_factor * (center_of_mass[0] - boid.position[0]), self.cohesion_factor * (center_of_mass[1] - boid.position[1]))
        return (0, 0)
    
    def calculate_separation(self, boid):
        """
        Calculate the separation force for a boid based on its neighbors.

        :param boid: The Boid instance to calculate the separation force for.
        :return: A tuple (fx, fy) representing the separation force.
        """
        separation_force = (0, 0)
        count = 0
        for other_boid in self.boid_array:
            if other_boid != boid:
                distance = math.sqrt((other_boid.position[0] - boid.position[0])**2 + (other_boid.position[1] - boid.position[1])**2)
                if distance < self.protection_distance:
                    separation_force = (separation_force[0] + (boid.position[0] - other_boid.position[0]), separation_force[1] + (boid.position[1] - other_boid.position[1]))
                    count += 1
        if count > 0:
            return (self.separation_factor * separation_force[0], self.separation_factor * separation_force[1])
        return (0, 0)
    
    def calculate_targetting(self, boid : Boid, target):
        """
        Calculate the targetting force for a boid based on its target.

        :param boid: The Boid instance to calculate the targetting force for.
        :param target: The target position (x, y) to calculate the targetting force towards.
        :return: A tuple (fx, fy) representing the targetting force.
        """
        
        
        distance = math.sqrt((target[0] - boid.position[0])**2 + (target[1] - boid.position[1])**2)
        detected = (distance < self.max_distance)
        
        # Close enough to the target 
        if detected:
            if boid.poi_position != target:
                boid.save_poi()
                boid.relay_boid = boid
            boid.poi_position = target
            boid.poi_time = boid.counter
            targetting_poi_force = (self.targetting_factor * (boid.poi_position[0] - boid.position[0]), self.targetting_factor * (boid.poi_position[1] - boid.position[1]))
            return targetting_poi_force
        
        # Not close enough to the target
        if boid.poi_position != None:
            distance_poi = math.sqrt((boid.poi_position[0] - boid.position[0])**2 + (boid.poi_position[1] - boid.position[1])**2)
            targetting_poi_force = (self.targetting_factor * (boid.poi_position[0] - boid.position[0]), self.targetting_factor * (boid.poi_position[1] - boid.position[1]))
            # Close enough to the poi
            if distance_poi < self.max_distance:
                boid.save_poi()
                boid.relay_boid = None
                boid.poi_position = None
                boid.poi_time = 0
                return (0, 0)
            # Not close enough to the poi
            else:
                return targetting_poi_force     
        return (0, 0)
    
    def render(self, screen):
        """
        Render all boids on the screen.
        """
        for boid in self.boid_array:
            boid.render(screen)
            if boid.leader and not self.render_model:
                # Draw circle around the leader boid for demo
                pygame.draw.circle(screen, 
                                   (0, 255, 255), 
                                   (int(boid.position[0]), int(boid.position[1])), 
                                   15,
                                   3)
            if self.render_model:
                if boid.poi_position != None and boid.relay_boid == boid:
                    # Draw a line to the target
                    pygame.draw.line(screen, (255, 0, 0), (int(boid.position[0]), int(boid.position[1])), (int(boid.poi_position[0]), int(boid.poi_position[1])), 1)
                elif boid.poi_position != None and boid.relay_boid != None:
                    # Draw a line to the relay boid
                    pygame.draw.line(screen, (255, 255, 0), (int(boid.position[0]), int(boid.position[1])), (int(boid.relay_boid.position[0]), int(boid.relay_boid.position[1])), 1)
                # Draw the poi position
                if boid.poi_position != None:
                    pygame.draw.circle(screen, 
                                    (255, 255, 0), 
                                    (int(boid.poi_position[0]), int(boid.poi_position[1])),
                                    5,
                                    5)

    def reset(self):
        """
        Reset the boids' positions and velocities.
        """
        for boid in self.boid_array:
            boid.reset_acceleration()
            
    def plot(self):
        """
        Plot
        self.collision_counts = [] # List to store the number of collisions
        self.target_positions = [] # List to store the target positions
        self.avg_distances = [] # List to store the average distances to the target
        self.found_poi_counts = [] # List to store the number of boids that found the target POI
        """
        # Plot the number of collisions over time
        plt.figure(figsize=(10, 5))
        plt.subplot(1, 3, 1)
        plt.plot(self.collision_counts, label='Collisions')
        plt.xlabel('Time')
        plt.ylabel('Number of Collisions')
        plt.title('Collisions Over Time')
        plt.legend()
        
        # Plot the average distance to the target over time
        plt.subplot(1, 3, 2)
        plt.plot(self.avg_distances, label='Average Distance to Target')
        plt.xlabel('Time')
        plt.ylabel('Average Distance')
        plt.title('Average Distance to Target Over Time')
        plt.legend()
        
        # Plot the number of boids that found a point of interest (POI) over time
        plt.subplot(1, 3, 3)
        plt.plot(self.found_poi_counts, label='Boids Found POI')
        plt.xlabel('Time')
        plt.ylabel('Number of Boids Found POI')
        plt.title('Boids Found POI Over Time')
        plt.legend()
        
        # Plot the target positions over time and the avg_poi_positions
        plt.figure(figsize=(10, 5))
        plt.subplot(1, 2, 1)
        plt.plot([pos[0] for pos in self.target_positions], label='Target X Position')
        plt.plot([pos[0] for pos in self.avg_poi_positions], label='Avg POI X Position')
        plt.xlabel('Time')
        plt.ylabel('X Position')
        plt.title('Target X Position Over Time')
        plt.subplot(1, 2, 2)
        plt.plot([pos[1] for pos in self.target_positions], label='Target Y Position')
        plt.plot([pos[1] for pos in self.avg_poi_positions], label='Avg POI Y Position')
        plt.xlabel('Time')
        plt.ylabel('Y Position')
        plt.title('Target Y Position Over Time')
        plt.legend()
        
        # Show the plots
        plt.show()