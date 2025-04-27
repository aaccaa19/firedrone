import simpy
import numpy as np
import random
import matplotlib.pyplot as plt
import fcl  # Python-FCL for collision detection
import time
from mpl_toolkits.mplot3d import Axes3D  # Import for 3D plotting
from mpl_toolkits.mplot3d import art3d  # Import for 3D patch handling

fires = []  # List to hold fire objects
drones = []  # List to hold drone objects
scattered_fuel = []  # List to hold scattered fuel objects

class Fuel:
    def __init__(self, env, durability, location, fuel_type="Generic"):
        self.env = env
        self.durability = durability
        self.location = (location[0], location[1], 0)  # Set Z-coordinate to 0 to place on the ground
        self.fuel_type = fuel_type
        self.is_burning = False  # Add a flag to track if the fuel is burning
        if self.fuel_type == "tree":
            self.height = random.uniform(1, 3)  # Random height for trees
        else:
            self.height = 0  # Non-tree fuel types have no height
        scattered_fuel.append(self)  # Add fuel to the global list

    def burn(self):
        self.is_burning = True  # Set the flag to True when burning starts
        while self.durability > 0:
            yield self.env.timeout(1)  # Burn for 1 time unit
            self.durability -= 1
            print(f"Time {self.env.now}: Fuel at {self.location} durability is now {self.durability}")
        print(f"Time {self.env.now}: Fuel at {self.location} is completely burned out!")
        if self.fuel_type == "house":
            print(f"Time {self.env.now}: House at {self.location} has burned down.")


class Fire:
    def __init__(self, env, location):
        self.env = env
        self.location = (location[0], location[1], 0)  # Set Z-coordinate to 0 to place on the ground
        fires.append(self)  # Add fire to the global list
        self.env.process(self.propagate())
        self.env.process(self.extinguish())  # Add process to extinguish fire after a random time

    def propagate(self):
        while True:
            yield self.env.timeout(2)  # Fire propagates every 2 time units
            new_location = (self.location[0] + random.randint(-2, 2),
                            self.location[1] + random.randint(-2, 2),
                            0)  # Ensure fire stays on the ground
            for fuel_obj in scattered_fuel:
                if fuel_obj.durability > 0 and not fuel_obj.is_burning:
                    distance = ((new_location[0] - fuel_obj.location[0]) ** 2 +
                                (new_location[1] - fuel_obj.location[1]) ** 2) ** 0.5
                    if distance <= 2.5:  # If within proximity, ignite the fuel
                        print(f"Time {self.env.now}: Fire at {self.location} ignites {fuel_obj.fuel_type} at {fuel_obj.location}.")
                        fuel_obj.is_burning = True  # Mark the fuel as burning
                        env.process(fuel_obj.burn())
                        Fire(env, location=fuel_obj.location)  # Create a new fire at the fuel's location

            # 30% chance to spread to a nearby random location
            if random.random() < 0.3:
                nearby_location = (self.location[0] + random.randint(-3, 3),
                                   self.location[1] + random.randint(-3, 3),
                                   0)  # Ensure fire stays on the ground
                print(f"Time {self.env.now}: Fire at {self.location} spreads to nearby location {nearby_location}.")
                Fire(env, location=nearby_location)

    def extinguish(self):
        yield self.env.timeout(random.randint(5, 15))  # Fire lasts for a random time between 5 and 15 time units
        print(f"Time {self.env.now}: Fire at {self.location} has been extinguished.")
        fires.remove(self)  # Remove fire from the global list


class Drone:
    def __init__(self, env, name, battery_life, wind_speed, start_location):
        self.env = env
        self.name = name
        self.battery_life = battery_life
        self.wind_speed = wind_speed
        self.location = (start_location[0], start_location[1], random.randint(0, 10))  # Add Z-coordinate
        self.start_location = self.location  # Store the starting location
        self.charging = False  # Track if the drone is charging
        drones.append(self)
        self.env.process(self.fly())

    def fly(self):
        while True:
            if self.battery_life <= 5 and not self.charging:  # Return to charge if battery is low
                print(f"Time {self.env.now}: {self.name} is returning to starting point to charge.")
                yield self.env.process(self.return_to_charge())

            if not self.charging:
                yield self.env.timeout(1)  # Fly for 1 time unit
                self.battery_life -= 1 + abs(self.wind_speed) * 0.1  # Wind affects battery usage
                self.location = (self.location[0] + random.uniform(-1, 1),
                                 self.location[1] + random.uniform(-1, 1),
                                 self.location[2] + random.uniform(-0.5, 0.5))  # Add Z-coordinate movement
                print(f"Time {self.env.now}: {self.name} is flying. Battery life: {self.battery_life:.2f}. Location: {self.location}")

    def return_to_charge(self):
        self.charging = True
        while self.location != self.start_location:
            yield self.env.timeout(1)  # Move towards the starting point
            self.location = (
                self.location[0] - (self.location[0] - self.start_location[0]) * 0.5,
                self.location[1] - (self.location[1] - self.start_location[1]) * 0.5,
                self.location[2] - (self.location[2] - self.start_location[2]) * 0.5
            )
            print(f"Time {self.env.now}: {self.name} is returning to charge. Location: {self.location}")

        print(f"Time {self.env.now}: {self.name} has reached the starting point and is charging.")
        yield self.env.timeout(5)  # Simulate charging time
        self.battery_life = 20  # Recharge the battery
        self.charging = False
        print(f"Time {self.env.now}: {self.name} has finished charging and is resuming exploration.")


def detect_collisions(drones, fires):
    """Safely detect collisions using FCL."""
    drone_objects = []
    fire_objects = []

    for drone in drones:
        try:
            if drone.battery_life <= 0:
                continue  # Skip drones with no battery
            assert len(drone.location) == 3
            pos = [float(drone.location[0]), float(drone.location[1]), float(drone.location[2])]
            drone_objects.append(fcl.CollisionObject(fcl.Sphere(0.5), fcl.Transform(pos)))
        except Exception as e:
            print(f"🚨 Error creating drone FCL object at location {drone.location}: {e}")

    for fire in fires:
        try:
            assert len(fire.location) == 3
            pos = [float(fire.location[0]), float(fire.location[1]), float(fire.location[2])]
            fire_objects.append(fcl.CollisionObject(fcl.Sphere(0.5), fcl.Transform(pos)))
        except Exception as e:
            print(f"🚨 Error creating fire FCL object at location {fire.location}: {e}")

    try:
        if not drone_objects or not fire_objects:
            return  # Skip collision detection if no valid objects

        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(drone_objects + fire_objects)
        manager.setup()

        def callback(o1, o2, data):
            result = fcl.CollisionResult()
            request = fcl.CollisionRequest()
            fcl.collide(o1, o2, request, result)
            if result.is_collision:
                print("🔥 Collision detected between objects!")

        manager.collide(callback, None)
    except Exception as e:
        print("❗️Error during collision detection:", e)


# Update visualization to support 3D
fig, ax = None, None
ax_3d = None  # Add a 3D axis
drone_scatter, fire_scatter, fuel_scatter = None, None, None

def initialize_plot():
    global fig, ax, ax_3d, drone_scatter, fire_scatter, fuel_scatter
    fig = plt.figure(figsize=(8, 8))
    ax_3d = fig.add_subplot(111, projection='3d')  # Create a 3D subplot
    ax_3d.set_xlim(-10, 10)
    ax_3d.set_ylim(-10, 10)
    ax_3d.set_zlim(0, 10)  # Add Z-axis limits
    ax_3d.set_xlabel("X-axis")
    ax_3d.set_ylabel("Y-axis")
    ax_3d.set_zlabel("Z-axis")
    ax_3d.set_title("Live Fire & Drone Simulation (3D)")
    drone_scatter = ax_3d.scatter([], [], [], c='blue', label='Drones')
    fire_scatter = ax_3d.scatter([], [], [], c='red', label='Fires')
    fuel_scatter = ax_3d.scatter([], [], [], c='green', label='Fuel')
    ax_3d.legend()
    plt.ion()  # Turn on interactive mode
    plt.show()

def update_plot(drones, fires, scattered_fuel):
    ax_3d.cla()  # Clear the 3D plot
    ax_3d.set_xlim(-10, 10)
    ax_3d.set_ylim(-10, 10)
    ax_3d.set_zlim(0, 10)
    ax_3d.set_xlabel("X-axis")
    ax_3d.set_ylabel("Y-axis")
    ax_3d.set_zlabel("Z-axis")
    ax_3d.set_title("Live Fire & Drone Simulation (3D)")

    # Plot drones
    for drone in drones:
        if drone.battery_life > 0:
            ax_3d.scatter(*drone.location, c='blue', label=drone.name if drone.battery_life > 0 else None)

    # Plot fires
    for fire in fires:
        x, y, z = fire.location
        ax_3d.bar3d(x, y, z, 0.5, 0.5, 4, color='red', label='Fire')

    # Plot scattered fuel with different shapes
    for fuel_obj in scattered_fuel:
        if fuel_obj.durability > 0:
            if fuel_obj.fuel_type == "tree":
                # Represent trees as cylinders with their specific height
                x, y, z = fuel_obj.location
                ax_3d.bar3d(x, y, 0, 0.5, 0.5, fuel_obj.height, color='darkgreen', label='Tree')
            elif fuel_obj.fuel_type == "house":
                # Represent houses as cubes
                x, y, z = fuel_obj.location
                ax_3d.bar3d(x, y, 0, 1, 1, 1, color='brown', label='House')
            elif fuel_obj.fuel_type == "bush":
                # Represent bushes as spheres
                x, y, z = fuel_obj.location
                u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
                bush_x = 0.5 * np.cos(u) * np.sin(v) + x
                bush_y = 0.5 * np.sin(u) * np.sin(v) + y
                bush_z = 0.5 * np.cos(v) + z
                ax_3d.plot_surface(bush_x, bush_y, bush_z, color='green', label='Bush')
        elif fuel_obj.durability == 0:  # Burned out objects become flat circles
            x, y, z = fuel_obj.location
            circle = plt.Circle((x, y), 0.5, color='black', label='Burned', alpha=0.5)
            ax_3d.add_patch(circle)
            art3d.pathpatch_2d_to_3d(circle, z=0, zdir="z")

    fig.canvas.draw()
    fig.canvas.flush_events()

    # Debugging logs to ensure visualization
    print(f"Fires visualized at: {[fire.location for fire in fires]}")
    print(f"Fuel objects visualized at: {[fuel.location for fuel in scattered_fuel if fuel.durability > 0]}")


def check_proximity_and_burn(env, scattered_fuel, fires):
    while True:
        yield env.timeout(1)  # Check every 1 time unit
        for fire in fires:
            for fuel_obj in scattered_fuel:
                if fuel_obj.durability > 0 and not fuel_obj.is_burning:  # Ensure fuel is not already burning
                    distance = ((fire.location[0] - fuel_obj.location[0]) ** 2 +
                                (fire.location[1] - fuel_obj.location[1]) ** 2 +
                                (fire.location[2] - fuel_obj.location[2]) ** 2) ** 0.5
                    if distance <= 1.5:  # If within proximity, ignite the fuel
                        print(f"Time {env.now}: Fire at {fire.location} ignites {fuel_obj.fuel_type} at {fuel_obj.location}.")
                        fuel_obj.is_burning = True  # Mark the fuel as burning
                        env.process(fuel_obj.burn())
                        new_fire = Fire(env, location=fuel_obj.location)
                        fires.append(new_fire)


def scatter_random_objects(env):
    """Scatter random objects like houses, bushes, and trees at the start of the simulation."""
    fuel_types = ["house", "bush", "tree"]
    for _ in range(50):  # Increased the number of random objects to 50
        location = (random.randint(-10, 10), random.randint(-10, 10))
        durability = random.randint(5, 15)
        fuel_type = random.choice(fuel_types)
        Fuel(env, durability, location, fuel_type)


# SimPy environment
env = simpy.Environment()

# Scatter random objects at the start of the simulation
scatter_random_objects(env)

# Initialize fuel, fire, and drones
fuel = Fuel(env, durability=10, location=(0, 0))
fires.clear()
initial_fire = Fire(env, location=(0, 0))

# Ensure the plot is initialized before updating it
initialize_plot()
update_plot(drones, fires, scattered_fuel)

drone1 = Drone(env, name="Drone 1", battery_life=20, wind_speed=2, start_location=(5, 5))
drone2 = Drone(env, name="Drone 2", battery_life=15, wind_speed=-1, start_location=(-5, -5))



# Start the fuel burning process
env.process(fuel.burn())

# Add the proximity check process to the simulation
env.process(check_proximity_and_burn(env, scattered_fuel, fires))

# Ensure the simulation runs for all 100 steps regardless of drone activity
for step in range(200):
    env.step()
    detect_collisions(drones, fires)
    update_plot(drones, fires, scattered_fuel)
    print(f"Simulation step {step + 1} completed.")