from math import *
from model.Car import Car
from model.Game import Game
from model.Move import Move
from model.World import World
from model.TileType import TileType

try:
    from debug_client import Color
except ImportError:
    pass


class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = 10000
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.previous = None

    def __lt__(self, other):
        return self.id < other.id

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

import heapq

def dijkstra(aGraph, start, target):
    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)

            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)


class MyStrategy:
    def __init__(self):
        try:
            from debug_client import DebugClient
        except ImportError: # no debug module, maybe running on the russianaicup.ru server
            self.debug = None
        else:
            self.debug = DebugClient()
            self.green = Color(r=0.0, g=1.0, b=0.0)

    def build_path(self, world: World):
        self.path = Graph()
        for x in range(world.width):
            for y in range(world.height):
                if (world.tiles_x_y[x][y] == TileType.EMPTY):
                    continue
                elif (world.tiles_x_y[x][y] == TileType.VERTICAL):
                    self.path.add_edge((x, y), (x, y+1), 1)
                    self.path.add_edge((x, y), (x, y-1), 1)
                elif (world.tiles_x_y[x][y] == TileType.HORIZONTAL):
                    self.path.add_edge((x, y), (x+1, y), 1)
                    self.path.add_edge((x, y), (x-1, y), 1)
                elif (world.tiles_x_y[x][y] == TileType.LEFT_BOTTOM_CORNER):
                    self.path.add_edge((x, y), (x, y - 1), 1)
                    self.path.add_edge((x, y), (x + 1, y), 1)
                elif (world.tiles_x_y[x][y] == TileType.LEFT_TOP_CORNER):
                    self.path.add_edge((x, y), (x, y + 1), 1)
                    self.path.add_edge((x, y), (x + 1, y), 1)
                elif (world.tiles_x_y[x][y] == TileType.RIGHT_TOP_CORNER):
                    self.path.add_edge((x, y), (x, y + 1), 1)
                    self.path.add_edge((x, y), (x - 1, y), 1)
                elif (world.tiles_x_y[x][y] == TileType.RIGHT_BOTTOM_CORNER):
                    self.path.add_edge((x, y), (x, y - 1), 1)
                    self.path.add_edge((x, y), (x - 1, y), 1)

        for v in self.path:
            print("vertex:", v)

    def move(self, me: Car, world: World, game: Game, move: Move):
        if world.tick < game.initial_freeze_duration_ticks:
            if world.tick == 0:
                self.worldWidth = world.width
                self.build_path(world)
                return
            if world.tick == 1:
                aFrom = (int(me.x//game.track_tile_size), int(me.y//game.track_tile_size))
                aTo = (me.next_waypoint_x, me.next_waypoint_y)
                print(aFrom, aTo)
                dijkstra(self.path, self.path.get_vertex(aFrom), self.path.get_vertex(aTo))

                target = self.path.get_vertex(aTo)
                path = [target.get_id()]
                shortest(target, path)
                print('The shortest path : %s' %(path[::-1]))

        nextWaypointX = (me.next_waypoint_x + 0.5) * game.track_tile_size
        nextWaypointY = (me.next_waypoint_y + 0.5) * game.track_tile_size

        angleToWaypoint = me.get_angle_to(nextWaypointX, nextWaypointY)
        speedModule = hypot(me.speed_x, me.speed_y)

        move.wheel_turn = angleToWaypoint  / pi
        move.engine_power = 0.8

        if (speedModule * speedModule * abs(angleToWaypoint) > 2.5 * 2.5 * pi):
            move.setBrake = True
            move.engine_power = -0.5

        # if world.tick > game.initial_freeze_duration_ticks:
        #    move.use_nitro = True
