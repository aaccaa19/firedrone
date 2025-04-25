import simpy
import numpy as np
import random
import matplotlib.pyplot as plt
import fcl  # Python-FCL for collision detection
import time

fires = []  # List to hold fire objects
drones = []  # List to hold drone objects

class Fuel:
    def __init__(self, env, durability, location):
        self.env = env
        self.durability = durability
        self.location = location

    def burn(self):
        while self.durability > 0:
            yield self.env.timeout(1)  # Burn for 1 time unit
            self.durability -= 1
            print(f"Time {self.env.now}: Fuel at {self.location} durability is now {self.durability}")
        print(f"Time {self.env.now}: Fuel at {self.location} is completely burned out!")


class Fire:
    def __init__(self, env, location, fuel):
        self.env = env
        self.location = location
        self.fuel = fuel
        fires.append(self)  # Add fire to the global list
        self.env.process(self.propagate())
        


    def propagate(self):
        while self.fuel.durability > 0:
            yield self.env.timeout(2)  # Fire propagates every 2 time units
            new_location = (self.location[0] + random.choice([-1, 1]),
                            self.location[1] + random.choice([-1, 1]))
            print(f"Time {self.env.now}: Fire propagates to {new_location}")
            # Simulate new fire at the new location
            new_fuel = Fuel(self.env, durability=random.randint(5, 10), location=new_location)
            self.env.process(new_fuel.burn())
            Fire(self.env, new_location, new_fuel)


class Drone:
    def __init__(self, env, name, battery_life, wind_speed, start_location):
        self.env = env
        self.name = name
        self.battery_life = battery_life
        self.wind_speed = wind_speed
        self.location = start_location
        drones.append(self)
        self.env.process(self.fly())

    def fly(self):
        while self.battery_life > 0:
            yield self.env.timeout(1)  # Fly for 1 time unit
            self.battery_life -= 1 + abs(self.wind_speed) * 0.1  # Wind affects battery usage
            self.location = (self.location[0] + random.uniform(-1, 1),
                             self.location[1] + random.uniform(-1, 1))
            print(f"Time {self.env.now}: {self.name} is flying. Battery life: {self.battery_life:.2f}. Location: {self.location}")
        print(f"Time {self.env.now}: {self.name} has run out of battery!")


def detect_collisions(drones, fires):
    """Safely detect collisions using FCL."""
    drone_objects = []
    fire_objects = []

    for drone in drones:
        try:
            assert len(drone.location) == 2
            pos = [float(drone.location[0]), float(drone.location[1]), 0.0]
            drone_objects.append(fcl.CollisionObject(fcl.Sphere(0.5), fcl.Transform(pos)))
        except Exception as e:
            print(f"🚨 Error creating drone FCL object at location {drone.location}: {e}")

    for fire in fires:
        try:
            assert len(fire.location) == 2
            pos = [float(fire.location[0]), float(fire.location[1]), 0.0]
            fire_objects.append(fcl.CollisionObject(fcl.Sphere(0.5), fcl.Transform(pos)))
        except Exception as e:
            print(f"🚨 Error creating fire FCL object at location {fire.location}: {e}")

    try:
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


def visualize(drones, fires):
    """Visualize the simulation using matplotlib."""
    plt.figure(figsize=(8, 8))
    for drone in drones:
        plt.scatter(*drone.location, c='blue', label=drone.name if drone.battery_life > 0 else None)
    for fire in fires:
        plt.scatter(*fire.location, c='red', label='Fire')
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Simulation Visualization")
    plt.legend()
    plt.show(block=False)
    plt.pause(1) 
    plt.close()

# Global variables for the plot elements
fig, ax = None, None
drone_scatter, fire_scatter = None, None

def initialize_plot():
    global fig, ax, drone_scatter, fire_scatter
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_title("Live Fire & Drone Simulation")
    drone_scatter = ax.scatter([], [], c='blue', label='Drones')
    fire_scatter = ax.scatter([], [], c='red', label='Fires')
    ax.legend()
    plt.ion()  # Turn on interactive mode
    plt.show()

def update_plot(drones, fires):
    drone_positions = np.array([drone.location for drone in drones if drone.battery_life > 0])
    fire_positions = np.array([fire.location for fire in fires])

    if len(drone_positions) > 0:
        drone_scatter.set_offsets(drone_positions)
    else:
        drone_scatter.set_offsets([])

    if len(fire_positions) > 0:
        fire_scatter.set_offsets(fire_positions)
    else:
        fire_scatter.set_offsets([])

    fig.canvas.draw()
    fig.canvas.flush_events()

# SimPy environment
env = simpy.Environment()

# Initialize fuel, fire, and drones
fuel = Fuel(env, durability=10, location=(0, 0))
fires.clear()
fire = Fire(env, location=(0, 0), fuel=fuel)
drone1 = Drone(env, name="Drone 1", battery_life=20, wind_speed=2, start_location=(5, 5))
drone2 = Drone(env, name="Drone 2", battery_life=15, wind_speed=-1, start_location=(-5, -5))



# Start the fuel burning process
env.process(fuel.burn())

# Run the simulation and visualize
initialize_plot()

for step in range(100):  # however many steps you want
    #input(f"\nPress Enter to run simulation step {step + 1}...")
    env.step()
    detect_collisions(drones, fires)
    update_plot(drones, fires)