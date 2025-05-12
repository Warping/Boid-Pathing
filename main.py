import pygame
import matplotlib.pyplot as plt
import boids

""" 
This is a simple simulation of boids using Pygame.

The boids are represented as triangles that move around the screen.

The simulation includes basic boid behaviors such as alignment, cohesion, and separation.

The boids are attracted to a target point, which is represented by a yellow circle.

The simulation includes 2 clusters of boids, one with a target and one without.

The boid_model is attracted to the leader of the boid_target_model.

The boid_target_model is not attracted to any target.

Yellow lines represent information about the boids' target relayed from another boid in the model

Red lines represent information about the boids' target relayed from a boid within range of the target

Statistics of the boids are plotted at the end of the simulation.
"""

def main():
    pygame.init()
    screen = pygame.display.set_mode((1400, 800))
    clock = pygame.time.Clock()

    # Boid parameters
    boid_parameters = {
        "max_speed": 4,
        "min_speed": 2,
        "max_acceleration": 0.5,
        "max_angular_velocity": 0.05,
        "min_angular_velocity": 0.0,
        "max_angular_acceleration": 0.1,
        "render_color": (100, 100, 100)
    }
    
    # Boid target parameters
    boid_target_parameters = {
        "max_speed": 4,
        "min_speed": 2,
        "max_acceleration": 0.5,
        "max_angular_velocity": 0.05,
        "min_angular_velocity": 0.0,
        "max_angular_acceleration": 0.1,
        "render_color": (0, 255, 255)
    }

    # Create a Boids instance with 100 boids
    boid_model = boids.Boids(screen, 
                             boid_count=100, 
                             align_factor=0.05, 
                             cohesion_factor=0.00005, 
                             separation_factor=0.5, 
                             targetting_factor=0.05,
                             max_distance=60,
                             protection_distance=35,
                             collision_distance=5,
                             render_model=True,
                             boid_parameters=boid_parameters)
    
    # Create a Boids instance with 2 target boids
    boid_target_model = boids.Boids(screen, 
                             boid_count=30, 
                             align_factor=0.05, 
                             cohesion_factor=0.00005, 
                             separation_factor=0.5, 
                             targetting_factor=0.0,
                             max_distance=200,
                             protection_distance=35,
                             collision_distance=5,
                             render_model=False,
                             boid_parameters=boid_target_parameters)
    
    
    # Main simulation loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                
                
        # Clear the screen
        screen.fill((0, 0, 0))

        # Update the target boids
        boid_target_model.update((0, 0))
        # Draw the target boids
        boid_target_model.render(screen)
        # Reset the target boids' acceleration
        boid_target_model.reset()
        
        # Get position of leader target
        for boid in boid_target_model.boid_array:
            if boid.leader:
                target_position = boid.position
                break
        
        # Update the boids
        boid_model.update(target_position)
        # Draw the boids
        boid_model.render(screen)
        # Reset the boids' acceleration
        boid_model.reset()
        # Update the display
        pygame.display.flip()
        clock.tick(60)
    # Plot the statistics of the boids
    boid_model.plot()
    
        
    pygame.quit()
if __name__ == "__main__":
    main()

