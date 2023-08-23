import numpy as np
from Bounds import Bounds

"""
QNode class is a node of Quadtree
    - bounds    : (Bounds) bounds of the node
    - capacity  :    (int) maximum number of points in the node
    - points    :   (list) list of points in the node
    - children  :   (dict) dictionary of children nodes [QNode]
    - is_divided:   (bool) True if the node is divided
"""


# noinspection SpellCheckingInspection
class QNode:
    def __init__(self, bounds, capacity=4):
        self.bounds = bounds
        self.capacity = capacity
        self.points = []
        self.children = {}
        self.is_divided = False

    # Si el nodo no está dividido, entonces divide el nodo en 4 hijos
    # Los límites de los hijos se calculan por los límites del nodo actual
    # Los hijos se almacenan en el diccionario 'children' con la clave 'NW', 'NE', 'SW', 'SE'
    # -> No retorna nada
    def subdivide(self):
        if self.is_divided:
            return

        min_x, max_x = self.bounds.min_x, self.bounds.max_x
        min_y, max_y = self.bounds.min_y, self.bounds.max_y
        center = self.bounds.center

        self.children['NW'] = QNode(Bounds(min_x, center[1], center[0], max_y))
        self.children['NE'] = QNode(Bounds(center[0], center[1], max_x, max_y))
        self.children['SW'] = QNode(Bounds(min_x, min_y, center[0], center[1]))
        self.children['SE'] = QNode(Bounds(center[0], min_y, max_x, center[1]))

        # insertar los puntos en las nuevas hojas
        for p in self.points:

            if self.children['NW'].bounds.contains(p.position):
                self.children['NW'].points.append(p)

            elif self.children['NE'].bounds.contains(p.position):
                self.children['NE'].points.append(p)

            elif self.children['SW'].bounds.contains(p.position):
                self.children['SW'].points.append(p)

            elif self.children['SE'].bounds.contains(p.position):
                self.children['SE'].points.append(p)

        self.points.clear()  # eliminar las referencias del nodo spliteado

        # verificar si es necesario hacer una llamada recursiva
        if len(self.children['NW'].points) > self.capacity:
            self.children['NW'].subdivide()

        elif len(self.children['NE'].points) > self.capacity:
            self.children['NE'].subdivide()

        elif len(self.children['SW'].points) > self.capacity:
            self.children['SW'].subdivide()

        elif len(self.children['SE'].points) > self.capacity:
            self.children['SE'].subdivide()

        self.is_divided = True

    # Devuelve True si el nodo está dividido
    def is_leaf(self):
        return not self.is_divided

    # Query circle
    # Si el círculo no se intersecta con los límites del nodo, entonces no se hace nada
    # Si el nodo es una hoja, entonces se comprueba si los puntos del nodo están dentro del círculo
    # Si el nodo no es una hoja, entonces se llama a la función query_circle de cada hijo
    # Los puntos encontrados se almacenan en la lista 'found_points'
    #   - point: (np.array) centro del círculo
    #   - radius: (float) radio del círculo 
    #   - found_points: (list) donde se almancean los puntos encontrados
    #   -> No retorna nada
    def query_circle(self, point, radius, found_points):

        if self.is_divided:
            for direction in self.children:
                if self.children[direction].bounds.intersects_circle(point, radius):
                    self.children[direction].query_circle(point, radius, found_points)
        else:
            for cantidate in self.points:
                if np.linalg.norm(cantidate.position - point) <= radius:
                    found_points.append(cantidate)

    # Devuelve todos los nodos del árbol
    def nodes(self):
        all_nodes = [self]
        if not self.is_leaf():
            for child in self.children.values():
                all_nodes.extend(child.nodes())
        return all_nodes

    # Test: si el nodo es válido
    def is_valid(self):
        if len(self.points) > self.capacity:
            return False
        if self.is_leaf():
            return True
        for child in self.children.values():
            if not self.bounds.contains_bounds(child.bounds):
                return False
            if not child.is_valid():
                return False
        return True

    def insert(self, obj):
        self.points.append(obj)


"""
Quadtree class
    - root: (QNode) root node of the quadtree
"""


class Quadtree:
    def __init__(self, bounds, capacity=4):
        self.root = QNode(bounds, capacity)

    def insert_r(self, node, obj, point):

        if not node.is_divided:
            node.insert(obj)
            if len(node.points) > self.root.capacity:
                node.subdivide()
            return True

        else:
            for direction in node.children:
                if node.children[direction].bounds.contains(point):
                    self.insert_r(node.children[direction], obj, point)
                    return True

    # Inserta un objeto en el árbol
    # Si el objeto no está dentro de los límites del árbol, entonces no se hace nada
    # Si el objeto está dentro de los límites del árbol, entonces se inserta en el nodo correspondiente
    # Si el nodo tiene más puntos que su capacidad, entonces se divide el nodo
    #   - obj   :   (object) objecto a insertar
    #   - point : (np.array) posición del objeto
    #   - return:     (bool) True si el objeto se insertó correctamente
    def insert(self, obj, point):
        return self.insert_r(self.root, obj, point)

    # Devuelve todos los nodos del árbol
    def nodes(self):
        return self.root.nodes()

    # Query circle
    # Si el círculo no se intersecta con los límites del árbol, entonces no se hace nada
    # Si el círculo se intersecta con los límites del árbol, entonces se llama a la función query_circle del nodo raíz
    #   - point: (np.array) centro del círculo
    #   - radius: (float) radio del círculo
    #   - return: (list) lista de objetos encontrados
    def query_circle(self, point, radius):
        found_points = []
        self.root.query_circle(point, radius, found_points)
        return found_points

    # Limpia el árbol
    def clear(self):
        self.root = QNode(self.root.bounds, self.root.capacity)

    # Test: si el árbol es válido
    def is_valid(self):
        return self.root.is_valid()
