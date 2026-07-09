import math
import heapq

class World:
    START_NODE = 'START'
    BASKET_NODE = 'BASKET'
    TARGET_NODES = ['T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8', 'T9', 'T10']

    def __init__(self, width=5, height=5):
        self.width = width
        self.height = height

        self.nodes = {
            # Principal nodes
            'START':  (0.5, 0.5),
            'BASKET': (4.5, 1.0),
            'T1':     (1.0, 1.5),
            'T2':     (2.0, 0.5),
            'T3':     (3.5, 0.5),
            'T4':     (4.5, 2.0),
            'T5':     (2.0, 2.0),
            'T6':     (3.5, 2.5),
            'T7':     (2.5, 3.0),
            'T8':     (2.0, 4.5),
            'T9':     (1.0, 4.0),
            'T10':    (4.0, 4.5),

            # Grid nodes
            # walls (xmin, ymin, xmax, ymax) with radius margin for
            # W1 = (2.0, 1.0), (3.5, 1.0) is a wall
            # W2 = (2.5, 3.5), (5.0, 3.5) is a wall
            # W3 = (0.5, 2.0), (1.5, 3.0) is a box
            # Auxiliar Nodes:
            'N8':     (1.5, 1.0),
            'N9':     (4.0, 1.0),
            'N12':    (1.5, 1.5),
            'N14':    (2.5, 1.5),
            'N17':    (4.0, 1.5),
            'N33':    (0.5, 3.5),
            'N35':    (1.5, 3.5),
            'N36':    (2.0, 3.5),
        }
        '''
            'N1': (1.0, 0.5),
            'N2': (1.5, 0.5),
            'N3': (2.5, 0.5),
            'N4': (3.0, 0.5),
            'N5': (4.0, 0.5),
            'N6': (4.5, 0.5),

            'N7': (0.5, 1.0),
            'N8': (1.5, 1.0),
            'N9': (4.0, 1.0),

            'N10': (0.5, 1.5),
            'N12': (1.5, 1.5),
            'N13': (2.0, 1.5),
            'N14': (2.5, 1.5),
            'N15': (3.0, 1.5),
            'N16': (3.5, 1.5),
            'N17': (4.0, 1.5),
            'N18': (4.5, 1.5),
    
            'N19': (2.5, 2.0),
            'N20': (3.0, 2.0),
            'N21': (3.5, 2.0),
            'N22': (4.0, 2.0),

            'N23': (2.0, 2.5),
            'N24': (2.5, 2.5),
            'N25': (3.0, 2.5),
            'N26': (4.0, 2.5),
            'N27': (4.5, 2.5),

            'N28': (2.0, 3.0),
            'N29': (3.0, 3.0),
            'N30': (3.5, 3.0),
            'N31': (4.0, 3.0),
            'N32': (4.5, 3.0),

            'N33': (0.5, 3.5),
            'N34': (1.0, 3.5),
            'N35': (1.5, 3.5),
            'N36': (2.0, 3.5),

            'N37': (0.5, 4.0),
            'N38': (1.5, 4.0),
            'N39': (2.0, 4.0),
            'N40': (2.5, 4.0),
            'N41': (3.0, 4.0),
            'N42': (3.5, 4.0),
            'N43': (4.0, 4.0),
            'N44': (4.5, 4.0),

            'N45': (0.5, 4.5),
            'N46': (1.0, 4.5),
            'N47': (1.5, 4.5),
            'N48': (2.5, 4.5),
            'N49': (3.0, 4.5),
            'N50': (3.5, 4.5),
            'N51': (4.5, 4.5)
        '''
        
        # ── Edges ─────────────────────────────────────
        self.graph = {node: {} for node in self.nodes}

        edges = [
            ('START', 'N8'),
            ('START', 'T2'),
            ('T2',   'T3'),
            ('T3',   'N9'),
            ('N9',   'BASKET'),

            ('START', 'T1'),
            ('N8',   'T1'),
            ('N8',   'N12'),
            ('T1',   'N12'),
            ('N12',  'T5'),
            ('T1',   'T5'),
            ('N17',  'T4'),

            ('T7',  'BASKET'),
            ('T6',  'BASKET'),
            ('T5',  'BASKET'),
            ('N9',   'N17'),
            ('T5',   'T6'),
            ('T5',   'T7'),
            ('T4',   'T6'),
            ('T6',   'T7'),
            ('BASKET','T4'),

            ('T1',   'N33'), 
            ('N33',  'N35'),
            ('N35',  'N36'),
            ('N33',  'T9'),
            ('N36',  'T7'),
            ('N36',  'T8'),
            ('T7',   'N36'),
            ('T9',   'N35'),
            ('T9',   'T8'),
            ('T8',   'T10'),
            ('N14',  'T5'), 
            ('N14',  'T6'), 
            ('N14',  'T7'),  
            ('N14',  'N17'), 
            ('N14',  'N12'),
            ('N14',  'T4'),
        ]

        for a, b in edges:
            p1 = self.nodes[a]
            p2 = self.nodes[b]
            self.graph[a][b] = math.dist(p1, p2)   
            self.graph[b][a] = math.dist(p2, p1)

    def distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def get_node_coords(self, name):
        return self.nodes[name]


class DijkstraPlanner:
    def __init__(self, world: World):
        self.world = world

    def plan(self, start_node: str, end_node: str):
        graph = self.world.graph
        distances = {node: float('inf') for node in graph}
        previous = {node: None for node in graph}
        distances[start_node] = 0.0
        pq = [(0.0, start_node)]

        while pq:
            current_dist, current_node = heapq.heappop(pq)
            if current_dist > distances[current_node]:
                continue
            if current_node == end_node:
                break
                
            # Verify if node exists in the graph before accessing its neighbors
            if current_node in graph:
                for neighbor, weight in graph[current_node].items():
                    d = current_dist + weight
                    if d < distances[neighbor]:
                        distances[neighbor] = d
                        previous[neighbor] = current_node
                        heapq.heappush(pq, (d, neighbor))

        path_names = []
        node = end_node
        while node is not None:
            path_names.insert(0, node)
            node = previous[node]

        if not path_names or path_names[0] != start_node:
            return [], float('inf')

        path_coords = [self.world.get_node_coords(n) for n in path_names]
        return path_coords, distances[end_node]