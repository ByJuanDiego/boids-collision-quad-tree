import numpy as np
from Quadtree import Quadtree

"""
Boid algorithm class
    - position : (np.array) position of the boid
    - velocity : (np.array) velocity of the boid
    - max_speed:    (float) maximum speed of the boid
    - max_force:    (float) maximum force of the boid
    - separation_distance: (float) distance for separation
    - alignment_distance : (float) distance for alignment
    - cohesion_distance  : (float)  distance for cohesion
"""


class Boid:
    def __init__(self, position, velocity, max_speed, max_force, separation_distance, alignment_distance,
                 cohesion_distance):
        self.position = position
        self.velocity = velocity
        self.max_speed = max_speed
        self.max_force = max_force
        self.separation_distance = separation_distance
        self.alignment_distance = alignment_distance
        self.cohesion_distance = cohesion_distance

    # Separation force Si la distancia entre el boid y el vecino es menor que la distancia de separación, entonces se
    # añade la fuerza de separación La fuerza de separación es la dirección desde el vecino al boid La fuerza de
    # separación se limita a la magnitud de la fuerza máxima Esto evita que los boids se amontonen - neighbors:     (
    # list) lista de vecinos - return   : (np.array) fuerza de separación
    def separation(self, neighbors, epsilon=0.0000001):
        steering = np.zeros(2)
        count = 0

        for boid in neighbors:
            delta_r = self.position - boid.position
            distance = np.linalg.norm(delta_r)
            if distance < self.separation_distance:
                steering += delta_r / (distance + epsilon)
                count += 1

        if count > 0:
            steering /= count
            return self._limit_force(steering)

        return steering

    # Alignment force Si la distancia entre el boid y el vecino es menor que la distancia de alineación, entonces se
    # añade la fuerza de alineación La fuerza de alineación es la dirección de la velocidad del vecino La fuerza de
    # alineación se limita a la magnitud de la fuerza máxima Esto evita que los boids se muevan en direcciones
    # opuestas - neighbors:     (list) lista de vecinos - return   : (np.array) fuerza de alineación
    def alignment(self, neighbors):
        steering = np.zeros(2)
        count = 0

        for boid in neighbors:
            delta_r = boid.position - self.position
            distance = np.linalg.norm(delta_r)
            if distance < self.alignment_distance:
                steering += (boid.velocity - self.velocity)
                count += 1

        if count > 0:
            steering /= count
            steering = self._limit_force(steering)
        return steering

    # Cohesion force Si la distancia entre el boid y el vecino es menor que la distancia de cohesión, entonces se
    # añade la fuerza de cohesión La fuerza de cohesión es la dirección desde el boid al vecino La fuerza de cohesión
    # se limita a la magnitud de la fuerza máxima Esto evita que los boids se muevan en direcciones opuestas -
    # neighbors:     (list) lista de vecinos - return   : (np.array) fuerza de cohesión
    def cohesion(self, neighbors):
        steering = np.zeros(2)
        count = 0

        for boid in neighbors:
            delta_r = boid.position - self.position
            distance = np.linalg.norm(delta_r)
            if distance < self.cohesion_distance:
                steering += delta_r
                count += 1

        if count > 0:
            steering /= count
            steering = self._limit_force(steering)
        return steering

    # Actualiza la posición y la velocidad del boid
    # La velocidad se limita a la magnitud de la velocidad máxima
    # Si el boid está fuera de los límites, entonces se hace rebotar (llama a la funcion bounce)
    #   - separation_force: (np.array) fuerza de separación
    #   - alignment_force : (np.array) fuerza de alineación
    #   - cohesion_force  : (np.array) fuerza de cohesión
    #   - bounds          : (Bounds) límites
    #   -> No retorna nada
    def update(self, separation_force, alignment_force, cohesion_force, bounds):
        self.velocity += (separation_force + alignment_force + cohesion_force)
        self.velocity = self._limit_speed(self.velocity)
        self.position += self.velocity
        self.bounce(bounds)

    # Hace rebotar el boid si está fuera de los límites
    #   - bounds: (Bounds) límites
    #   -> No retorna nada
    def bounce(self, bounds):
        min_x, min_y, max_x, max_y = bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y

        if self.position[0] <= min_x or self.position[0] >= max_x:
            self.velocity[0] = -2 * self.velocity[0]
            self.position[0] = np.clip(self.position[0], min_x, max_x)

        if self.position[1] <= min_y or self.position[1] >= max_y:
            self.velocity[1] = -2 * self.velocity[1]
            self.position[1] = np.clip(self.position[1], min_y, max_y)

    # Limita la magnitud de la fuerza
    #   - force : (np.array) fuerza
    #   - return: (np.array) fuerza limitada
    def _limit_force(self, force):
        force_norm = np.linalg.norm(force)
        if force_norm > self.max_force:
            force = (force / force_norm) * self.max_force
        return force

    # Limita la magnitud de la velocidad
    #  - velocity : (np.array) velocidad
    #  - return   : (np.array) velocidad limitada
    def _limit_speed(self, velocity):
        speed = np.linalg.norm(velocity)
        if speed > self.max_speed:
            velocity = (velocity / speed) * self.max_speed
        return velocity


"""
Boid simulation class
    - boids: list of Boid
    - bounds: Bounds
    - search_radius: float
    - quadtree_capacity: int
"""


class BoidSimulation:
    def __init__(self, boids, bounds, search_radius, quadtree_capacity):
        self.boids = boids
        self.bounds = bounds
        self.search_radius = search_radius
        self.quadtree_capacity = quadtree_capacity

        self.quadtree = Quadtree(self.bounds, capacity=self.quadtree_capacity)

    # Actualiza la posición y la velocidad de los boids
    # Primero se limpia el quadtree
    # Luego se insertan los boids en el quadtree
    # Para cada boid, se llama a la función query_circle del quadtree para obtener los vecinos
    # Luego se calcula la fuerza de separación, la fuerza de alineación y la fuerza de cohesión
    # Finalmente se llama a la función update del boid para actualizar la posición y la velocidad
    def step(self):
        self.quadtree.clear()
        for boid in self.boids:
            self.quadtree.insert(boid, boid.position)
        for boid in self.boids:
            neighbors = self.quadtree.query_circle(boid.position, self.search_radius)
            separation_force = boid.separation(neighbors)
            alignment_force = boid.alignment(neighbors)
            cohesion_force = boid.cohesion(neighbors)
            boid.update(separation_force, alignment_force, cohesion_force, self.bounds)
