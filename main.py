import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from Boids import Boid, BoidSimulation
from Bounds import Bounds


class Visualizer:
    def __init__(self, simulation, steps, interval):
        self.simulation = simulation
        self.steps = steps
        self.interval = interval * 1000  # Convert to milliseconds
        self.show_quadtree = False

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_xlim(self.simulation.bounds.min_x, self.simulation.bounds.max_x)
        self.ax.set_ylim(self.simulation.bounds.min_y, self.simulation.bounds.max_y)
        self.ax.set_aspect('equal')
        self.ax.autoscale(False)
        self.ax.axis('off')
        self.button_ax = plt.axes([0.7, 0.025, 0.15, 0.04])
        self.button = Button(self.button_ax, "Off")
        self.button.on_clicked(self._toggle_quadtree)

        self.anim = FuncAnimation(self.fig, self._animate, frames=self.steps, interval=self.interval, blit=False,
                                  repeat=False)
        plt.show()

    def _toggle_quadtree(self, event):
        self.show_quadtree = not self.show_quadtree
        self.button.label.set_text("On" if self.show_quadtree else "Off")

    def _animate(self, frame):
        self.ax.clear()
        self.ax.axis('off')
        self.ax.set_xlim(self.simulation.bounds.min_x, self.simulation.bounds.max_x)
        self.ax.set_ylim(self.simulation.bounds.min_y, self.simulation.bounds.max_y)

        for boid in self.simulation.boids:
            self.ax.plot(boid.position[0], boid.position[1], "bo", markersize=1)

        if self.show_quadtree:
            for node in self.simulation.quadtree.nodes():
                if node.is_leaf():
                    self.ax.plot(
                        [node.bounds.min_x, node.bounds.min_x, node.bounds.max_x, node.bounds.max_x, node.bounds.min_x],
                        [node.bounds.min_y, node.bounds.max_y, node.bounds.max_y, node.bounds.min_y, node.bounds.min_y],
                        "r-"
                    )

        self.simulation.step()


# Define los parámetros de la simulación
num_boids = 100  # Número de boids
bounds = Bounds(0, 0, 100, 100)  # Límites del espacio
search_radius = 15  # Radio de búsqueda
quadtree_capacity = 4  # Capacidad máxima de puntos en un nodo
steps = 1000  # Número de frames en la simulación
interval = 0.03  # Intervalo de tiempo entre cada frame
max_speed = 5  # Velocidad máxima
max_force = 0.3  # Fuerza máxima
separation_distance = 15  # Distancia de separación
alignment_distance = 10  # Distancia de alineación
cohesion_distance = 10  # Distancia de cohesión
max_initial_speed = 2  # Velocidad inicial máxima

# Inicializa los boids de forma aleatoria
boids = []
for _ in range(num_boids):
    position = np.random.rand(2) * [bounds.width, bounds.height]
    velocity = (np.random.rand(2) * max_initial_speed - 1) * max_initial_speed

    boid = Boid(position, velocity, max_speed, max_force, separation_distance, alignment_distance, cohesion_distance)
    boids.append(boid)

# Crea la simulación
simulation = BoidSimulation(boids, bounds, search_radius, quadtree_capacity)
visualizer = Visualizer(simulation, steps=steps, interval=interval)
