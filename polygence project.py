import simpy
import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.patches as patches
import gymnasium as gym 
import gymnasium_robotics
from gymnasium import spaces

# Global lists for simulation objects
fires, drones, scattered_fuel, helicopters = [], [], [], []

class Fuel:
    def __init__(self, env, durability, location, fuel_type="Generic"):
        self.env = env
        self.durability = durability
        self.location = (location[0], location[1], 0)  # Ensure on ground
        self.fuel_type = fuel_type
        self.is_burning = False
        self.height = random.uniform(1, 3) if fuel_type == "tree" else 0
        scattered_fuel.append(self)

    def burn(self):
        self.is_burning = True
        while self.durability > 0:
            yield self.env.timeout(1)
            self.durability -= 1
            print(f"Time {self.env.now}: Fuel at {self.location} durability is now {self.durability}")
        print(f"Time {self.env.now}: Fuel at {self.location} is completely burned out!")
        for fire in list(fires):
            if fire.location == self.location:
                fire.stop_fire()
                fires.remove(fire)

class Fire:
    def __init__(self, env, location, durability):
        # Ensure fire is not on any road
        while location in main_road_coordinates or location in side_road_coordinates or any(location in road for road in additional_road_coordinates):
            location = (random.randint(-10, 10), random.randint(-10, 10), 0)
        self.env = env
        self.location = (location[0], location[1], 0)
        self.durability = durability
        fires.append(self)
        self.env.process(self.propagate())
        self.env.process(self.extinguish())

    def propagate(self):
        while self.durability > 0:
            yield self.env.timeout(2)
            new_location = (self.location[0] + random.randint(-2, 2),
                            self.location[1] + random.randint(-2, 2), 0)
            for fuel_obj in scattered_fuel:
                if fuel_obj.durability > 0 and not fuel_obj.is_burning:
                    distance = ((new_location[0] - fuel_obj.location[0]) ** 2 +
                                (new_location[1] - fuel_obj.location[1]) ** 2) ** 0.5
                    if distance <= 2.5:
                        print(f"Time {self.env.now}: Fire at {self.location} ignites {fuel_obj.fuel_type} at {fuel_obj.location}.")
                        fuel_obj.is_burning = True
                        env.process(fuel_obj.burn())
                        Fire(env, location=fuel_obj.location, durability=fuel_obj.durability if fuel_obj.fuel_type in ["bush", "tree", "house"] else 5)
            if random.random() < 0.3:
                nearby_location = (self.location[0] + random.randint(-3, 3),
                                   self.location[1] + random.randint(-3, 3), 0)
                Fire(env, location=nearby_location, durability=5)

    def extinguish(self):
        while self.durability > 0:
            yield self.env.timeout(1)
            self.durability -= 1
        fires.remove(self)

    def stop_fire(self):
        self.durability = 0

class Drone:
    def __init__(self, env, name, battery_life, wind_speed, start_location):
        self.env = env
        self.name = name
        self.battery_life = battery_life
        self.wind_speed = wind_speed
        self.location = (start_location[0], start_location[1], random.randint(0, 10))
        self.start_location = start_location
        self.charging = False
        drones.append(self)
        self.env.process(self.fly())

    def fly(self):
        while True:
            if self.battery_life <= 5 and not self.charging:
                yield self.env.process(self.return_to_charge())
            if not self.charging:
                # Increase altitude after leaving the drone base
                if self.location[2] < 5:  # Target altitude is 5 units
                    self.location = (self.location[0], self.location[1], self.location[2] + 1)
                    print(f"Time {self.env.now}: {self.name} is increasing altitude. Current altitude: {self.location[2]}")
                else:
                    # Avoid collisions with objects
                    for fuel_obj in scattered_fuel:
                        distance = ((self.location[0] - fuel_obj.location[0]) ** 2 +
                                    (self.location[1] - fuel_obj.location[1]) ** 2 +
                                    (self.location[2] - fuel_obj.location[2]) ** 2) ** 0.5
                        if distance < 1.5:  # If too close to an object, adjust position
                            self.location = (
                                self.location[0] + random.uniform(-1, 1),
                                self.location[1] + random.uniform(-1, 1),
                                self.location[2]
                            )
                            print(f"Time {self.env.now}: {self.name} is avoiding collision. New location: {self.location}")
                            break
                    # Normal flight behavior
                    yield self.env.timeout(1)
                    self.battery_life -= 1 + abs(self.wind_speed) * 0.1
                    self.location = (
                        self.location[0] + random.uniform(-1, 1),
                        self.location[1] + random.uniform(-1, 1),
                        self.location[2]
                    )
                    print(f"Time {self.env.now}: {self.name} is flying. Battery life: {self.battery_life:.2f}. Location: {self.location}")

    def return_to_charge(self):
        self.charging = True
        while self.location != self.start_location:
            yield self.env.timeout(1)
            self.location = (
                self.location[0] - (self.location[0] - self.start_location[0]) * 0.5,
                self.location[1] - (self.location[1] - self.start_location[1]) * 0.5,
                self.location[2] - (self.location[2] - self.start_location[2]) * 0.5
            )
        yield self.env.timeout(5)
        self.battery_life = 20
        self.charging = False

class DroneBase:
    def __init__(self, location):
        self.location = (location[0], location[1], 0)  # Ensure the base is on the ground
        print(f"Drone base initialized at location {self.location}.")

class Helicopter:
    def __init__(self, env, name, start_location):
        self.env = env
        self.name = name
        self.location = (start_location[0], start_location[1], 5)  # Start at z=5
        self.radius = 0.5  # Represent helicopters as small spheres
        helicopters.append(self)
        self.env.process(self.fly_to_fire())

    def fly_to_fire(self):
        while True:
            if fires:
                # Find the nearest fire
                nearest_fire = min(fires, key=lambda fire: ((self.location[0] - fire.location[0]) ** 2 +
                                                            (self.location[1] - fire.location[1]) ** 2 +
                                                            (self.location[2] - fire.location[2]) ** 2) ** 0.5)
                # Move towards the fire at a constant speed of 2 units per step
                direction = (
                    nearest_fire.location[0] - self.location[0],
                    nearest_fire.location[1] - self.location[1],
                    nearest_fire.location[2] - self.location[2]
                )
                magnitude = (direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2) ** 0.5
                if magnitude > 0:
                    # Calculate step size (don't overshoot)
                    step_size = min(2, magnitude)  # 2 units max per step
                    
                    # Move towards the fire
                    self.location = (
                        self.location[0] + step_size * direction[0] / magnitude,
                        self.location[1] + step_size * direction[1] / magnitude,
                        self.location[2]  # Maintain altitude at z=5
                    )
                print(f"Time {self.env.now}: {self.name} is flying towards fire at {nearest_fire.location}. Current location: {self.location}")

                # Check if the helicopter is at the fire's location
                if magnitude <= 2:
                    print(f"Time {self.env.now}: {self.name} is extinguishing fire at {nearest_fire.location}.")
                    nearest_fire.stop_fire()
                    fires.remove(nearest_fire)
            else:
                # Fly randomly if no fires are present
                direction = (random.uniform(-1, 1), random.uniform(-1, 1), 0)
                magnitude = (direction[0] ** 2 + direction[1] ** 2) ** 0.5
                self.location = (
                    self.location[0] + 2 * direction[0] / magnitude,
                    self.location[1] + 2 * direction[1] / magnitude,
                    self.location[2]
                )
                print(f"Time {self.env.now}: {self.name} is flying randomly. Current location: {self.location}")
            yield self.env.timeout(1)

fig, ax_3d = None, None
main_road_coordinates = [(x, 0) for x in range(-10, 11)]  # Main road along the X-axis
side_road_coordinates = [(0, y) for y in range(-5, 6)]  # Side road along the Y-axis branching from the main road

# Increase the distance of parallel roads
additional_road_coordinates = [
    [(x, 6) for x in range(-10, 11)],  # Parallel road further above the main road
    [(x, -6) for x in range(-10, 11)],  # Parallel road further below the main road
    [(6, y) for y in range(-5, 6)],  # Vertical road further to the right of the side road
    [(-6, y) for y in range(-5, 6)]  # Vertical road further to the left of the side road
]

def initialize_plot():
    global fig, ax_3d
    fig = plt.figure(figsize=(8, 8))
    ax_3d = fig.add_subplot(111, projection='3d')
    ax_3d.set_xlim(-10, 10)
    ax_3d.set_ylim(-10, 10)
    ax_3d.set_zlim(0, 10)
    plt.ion()
    plt.show()

def update_plot():
    ax_3d.cla()
    ax_3d.set_xlim(-10, 10)
    ax_3d.set_ylim(-10, 10)
    ax_3d.set_zlim(0, 10)

    # Draw the main road (wider)
    for x, y in main_road_coordinates:
        road_patch = patches.Rectangle((x - 1, y - 0.5), 2, 1, color='grey', alpha=0.7)  # Wider main road
        ax_3d.add_patch(road_patch)
        art3d.pathpatch_2d_to_3d(road_patch, z=0, zdir="z")

    # Draw the side road (narrower)
    for x, y in side_road_coordinates:
        road_patch = patches.Rectangle((x - 0.5, y - 0.5), 1, 1, color='grey', alpha=0.7)  # Narrower side road
        ax_3d.add_patch(road_patch)
        art3d.pathpatch_2d_to_3d(road_patch, z=0, zdir="z")

    # Draw additional roads
    for road in additional_road_coordinates:
        for x, y in road:
            road_patch = patches.Rectangle((x - 0.5, y - 0.5), 1, 1, color='grey', alpha=0.7)
            ax_3d.add_patch(road_patch)
            art3d.pathpatch_2d_to_3d(road_patch, z=0, zdir="z")

    for drone in drones:
        if drone.battery_life > 0:
            ax_3d.scatter(*drone.location, c='blue')
    for fire in fires:
        x, y, z = fire.location
        ax_3d.bar3d(x, y, z, 0.5, 0.5, 4, color='red')
    for fuel_obj in scattered_fuel:
        if fuel_obj.durability > 0:
            if fuel_obj.fuel_type == "tree":
                x, y, z = fuel_obj.location
                ax_3d.bar3d(x, y, 0, 0.5, 0.5, fuel_obj.height, color='darkgreen')
            elif fuel_obj.fuel_type == "house":
                x, y, z = fuel_obj.location
                ax_3d.bar3d(x, y, 0, 1, 1, 1, color='brown')
            elif fuel_obj.fuel_type == "bush":
                x, y, z = fuel_obj.location
                u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
                bush_x = 0.5 * np.cos(u) * np.sin(v) + x
                bush_y = 0.5 * np.sin(u) * np.sin(v) + y
                bush_z = 0.5 * np.cos(v) + z
                ax_3d.plot_surface(bush_x, bush_y, bush_z, color='green')
        elif fuel_obj.durability == 0:
            x, y, z = fuel_obj.location
            circle = plt.Circle((x, y), 0.5, color='black', alpha=0.5)
            ax_3d.add_patch(circle)
            art3d.pathpatch_2d_to_3d(circle, z=0, zdir="z")
    for helicopter in helicopters:
        x, y, z = helicopter.location
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
        sphere_x = helicopter.radius * np.cos(u) * np.sin(v) + x
        sphere_y = helicopter.radius * np.sin(u) * np.sin(v) + y
        sphere_z = helicopter.radius * np.cos(v) + z
        ax_3d.plot_surface(sphere_x, sphere_y, sphere_z, color='yellow', alpha=0.7)
    fig.canvas.draw()
    fig.canvas.flush_events()

# Ensure buildings are not placed on roads

def scatter_random_objects(env):
    placed_houses = set()  # Track placed house locations to avoid overlap

    for _ in range(100):
        durability = random.randint(5, 15)
        fuel_type = random.choice(["house", "bush", "tree"])

        if fuel_type == "house":
            # Place houses along the main road
            if random.random() < 0.5:
                x = random.choice(range(-10, 11))
                y = random.choice([-2, 2])  # Ensure 0.5 space between house and road
            else:
                # Place houses along the side road
                x = random.choice([-2, 2])  # Ensure 0.5 space between house and road
                y = random.choice(range(-5, 6))

            location = (x, y)

            # Ensure no overlap with existing houses and not on roads
            while location in placed_houses or location in main_road_coordinates or location in side_road_coordinates or any(location in road for road in additional_road_coordinates):
                if random.random() < 0.5:
                    x = random.choice(range(-10, 11))
                    y = random.choice([-2, 2])
                else:
                    x = random.choice([-2, 2])
                    y = random.choice(range(-5, 6))
                location = (x, y)

            placed_houses.add(location)
            Fuel(env, durability, location, fuel_type)

            # Add trees next to houses
            tree_locations = [
                (location[0] + 1, location[1]),
                (location[0] - 1, location[1]),
                (location[0], location[1] + 1),
                (location[0], location[1] - 1)
            ]
            for tree_location in tree_locations:
                if tree_location not in placed_houses and tree_location not in main_road_coordinates and tree_location not in side_road_coordinates and not any(tree_location in road for road in additional_road_coordinates):
                    # Ensure trees are not touching houses
                    if all(abs(tree_location[0] - house[0]) > 1 or abs(tree_location[1] - house[1]) > 1 for house in placed_houses):
                        Fuel(env, random.randint(5, 15), tree_location, "tree")

        elif fuel_type == "tree":
            # Ensure trees are not on the roads or touching houses
            location = (random.randint(-10, 10), random.randint(-10, 10))
            while location[1] == 0 or location[0] == 0 or location in main_road_coordinates or location in side_road_coordinates or any(location in road for road in additional_road_coordinates) or any(abs(location[0] - house[0]) <= 1 and abs(location[1] - house[1]) <= 1 for house in placed_houses):
                location = (random.randint(-10, 10), random.randint(-10, 10))
            Fuel(env, durability, location, fuel_type)

class FireDroneEnv(gym.Env):
    def __init__(self):
        super(FireDroneEnv, self).__init__()
        # Define the action space (e.g., move in x, y, z directions)
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=float)

        # Define the observation space (e.g., positions of drones, fires, etc.)
        self.observation_space = spaces.Dict({
            "drone_location": spaces.Box(low=-10, high=10, shape=(3,), dtype=float),
            "fire_locations": spaces.Box(low=-10, high=10, shape=(len(fires), 3), dtype=float),
            "fuel_locations": spaces.Box(low=-10, high=10, shape=(len(scattered_fuel), 3), dtype=float),
        })

        # Initialize the simulation environment
        self.env = simpy.Environment()
        self.drones = []
        self.fires = []
        self.fuel = scattered_fuel
        self.step_count = 0

    def reset(self):
        # Reset the simulation environment
        self.env = simpy.Environment()
        self.drones = []
        self.fires = []
        scatter_random_objects(self.env)
        self.drones.append(Drone(self.env, name="Drone 1", battery_life=20, wind_speed=2, start_location=(0, 0, 0)))
        self.fires.append(Fire(self.env, location=(0, 0), durability=10))
        self.step_count = 0

        # Return the initial observation
        return self._get_observation()

    def step(self, action):
        # Apply the action to the drone
        drone = self.drones[0]
        drone.location = (
            drone.location[0] + action[0],
            drone.location[1] + action[1],
            drone.location[2] + action[2]
        )

        # Step the simulation environment
        self.env.step()
        self.step_count += 1

        # Calculate reward (e.g., based on proximity to fires or extinguishing fires)
        reward = self._calculate_reward()

        # Check if the episode is done (e.g., all fires extinguished or max steps reached)
        done = len(self.fires) == 0 or self.step_count >= 200

        # Return the observation, reward, done flag, and additional info
        return self._get_observation(), reward, done, {}

    def render(self, mode="human"):
        # Update the plot for visualization
        update_plot()

    def _get_observation(self):
        # Return the current state of the environment
        return {
            "drone_location": self.drones[0].location,
            "fire_locations": [fire.location for fire in self.fires],
            "fuel_locations": [fuel.location for fuel in self.fuel],
        }

    def _calculate_reward(self):
        # Example reward function: negative distance to the nearest fire
        drone = self.drones[0]
        if self.fires:
            nearest_fire = min(self.fires, key=lambda fire: ((drone.location[0] - fire.location[0]) ** 2 +
                                                             (drone.location[1] - fire.location[1]) ** 2 +
                                                             (drone.location[2] - fire.location[2]) ** 2) ** 0.5)
            distance = ((drone.location[0] - nearest_fire.location[0]) ** 2 +
                        (drone.location[1] - nearest_fire.location[1]) ** 2 +
                        (drone.location[2] - nearest_fire.location[2]) ** 2) ** 0.5
            return -distance
        return 0

env = simpy.Environment()
scatter_random_objects(env)
Fuel(env, durability=10, location=(0, 0))
Fire(env, location=(0, 0), durability=10)
initialize_plot()

# Initialize the drone base
base_location = (0, 0, 0)
drone_base = DroneBase(location=base_location)

# Update drones to start at the drone base
Drone(env, name="Drone 1", battery_life=20, wind_speed=2, start_location=base_location)
Drone(env, name="Drone 2", battery_life=20, wind_speed=2, start_location=base_location)

# Initialize helicopters
for i in range(2):  # Add 2 helicopters
    helicopters.append(Helicopter(env, name=f"Helicopter {i+1}", start_location=(0, 0)))

for step in range(200):
    env.step()
    update_plot()
    print(f"Simulation step {step + 1} completed.")