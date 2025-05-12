# Boid Pathing Simulation

This project implements a Boid simulation based on Craig Reynolds' Boid algorithm. The simulation models the behavior of a flock of boids (bird-like agents) that exhibit realistic group movement through alignment, cohesion, and separation behaviors. The simulation is visualized using Pygame.

## Features

- **Boid Behavior**: Implements alignment, cohesion, separation, and targeting behaviors.
- **Customizable Parameters**: Allows users to configure boid parameters such as speed, acceleration, and angular velocity.
- **Visualization**: Real-time rendering of boids and their interactions.
- **Data Analysis**: Plots metrics such as average distance to target and collisions over time.

## Requirements

- Python 3.10 or higher
- Pygame
- Matplotlib

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd Boid-Pathing
   ```
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. Run the simulation:
   ```bash
   python3 main.py
   ```
2. Use your mouse to move the target. The boids will dynamically adjust their behavior to follow the target.
3. Press `ESC` or close the window to exit the simulation.

## File Structure

- `main.py`: Entry point for the simulation. Initializes the Pygame window and runs the main loop.
- `boid.py`: Defines the `Boid` class, which handles individual boid behavior.
- `boids.py`: Defines the `Boids` class, which manages the flock of boids and their interactions.

## Customization

You can customize the boid parameters in `main.py` by modifying the `boid_parameters` dictionary:

```python
boid_parameters = {
    "max_speed": 4,  # Maximum speed of the boids
    "min_speed": 0.8,  # Minimum speed of the boids
    "max_acceleration": 0.2,  # Maximum acceleration of the boids
    "max_angular_velocity": 0.05,  # Maximum angular velocity of the boids
    "min_angular_velocity": 0.0,  # Minimum angular velocity of the boids
    "max_angular_acceleration": 0.2  # Maximum angular acceleration of the boids
}
```

## Example Output

The simulation visualizes the boids as triangles that move and interact in real-time. Metrics such as average distance to the target and collisions are plotted after the simulation ends.

Enjoy experimenting with the Boid Pathing Simulation!