# 6.0002 Problem Set 5
# Graph optimization
# Name:
# Collaborators:
# Time:

#
# Finding shortest paths through MIT buildings
#
import unittest
from graph import Digraph, Node, WeightedEdge

#
# Problem 2: Building up the Campus Map
#
# Problem 2a: Designing your graph
#
# What do the graph's nodes represent in this problem? Buildings
# What do the graph's edges represent? Path between two buildings
# Where are the distances represented? first weight on the edge is total distance
# second weight on the edge is total distance outside
#


# Problem 2b: Implementing load_map
def load_map(map_filename):
    """
    Parses the map file and constructs a directed graph

    Parameters:
        map_filename : name of the map file

    Assumes:
        Each entry in the map file consists of the following four positive
        integers, separated by a blank space:
            From To TotalDistance DistanceOutdoors
        e.g.
            32 76 54 23
        This entry would become an edge from 32 to 76.

    Returns:
        a Digraph representing the map
    """

    # TODO
    print("Loading map from file...")
    inFile = open(map_filename, 'r')
    
    my_edges = []
    my_nodes = []
    for line in inFile:
        #line will equal "32 76 54 23"
        #split line into src, dest, total_dist, outside_dist
        node_list = line.split(" ")
        node_list[3] = node_list[3].replace('\n', '')
        
        #first two numbers are src and dest nodes 
        src = Node(node_list[0])
        dest = Node(node_list[1])
        
        #add all unique nodes to node list
        if src not in my_nodes: 
            my_nodes.append(src)
        if dest not in my_nodes: 
            my_nodes.append(dest)
            
        #3rd number is total distance weight
        #4th number is outdoor distance weight
        total_dist = node_list[2]
        outside_dist = node_list[3]    
        e1 = WeightedEdge(src, dest, total_dist, outside_dist)
        my_edges.append(e1)

    my_digraph = Digraph()
    for node in my_nodes: 
          my_digraph.add_node(node)
    
    for edge in my_edges: 
        my_digraph.add_edge(edge)
    print("Digraph created!")
    #print(my_digraph)
    return my_digraph


#
# Problem 3: Finding the Shorest Path using Optimized Search Method
#
# Problem 3a: Objective function
#
# What is the objective function for this problem? What are the constraints?
# objective function: minimize total distance travelled between two points
# constraints: stay under max_dist_outdoors & max_total_dist
#

# Problem 3b: Implement get_best_path
def get_best_path(digraph, start, end, path, max_dist_outdoors, best_dist,
                  best_path):
    """
    Finds the shortest path between buildings subject to constraints.

    Parameters:
        digraph: Digraph instance
            The graph on which to carry out the search
        start: string
            Building number at which to start
        end: string
            Building number at which to end
        path: list composed of [[list of strings], int, int]
            Represents the current path of nodes being traversed. Contains
            a list of node names, total distance traveled, and total
            distance outdoors.
        max_dist_outdoors: int
            Maximum distance spent outdoors on a path
        best_dist: int
            The smallest distance between the original start and end node
            for the initial problem that you are trying to solve
        best_path: list of strings
            The shortest path found so far between the original start
            and end node.

    Returns:
        A tuple with the shortest-path from start to end, represented by
        a list of building numbers (in strings), [n_1, n_2, ..., n_k],
        where there exists an edge from n_i to n_(i+1) in digraph,
        for all 1 <= i < k and the distance of that path.

        If there exists no path that satisfies max_total_dist and
        max_dist_outdoors constraints, then return None.
    """
    # TODO
    """
  if start and end are not valid nodes: raise an error 
  elif start and end are the same node: 
      update the global variables appropriately 
      else: for all the child nodes of start construct 
      a path including that node recursively solve the 
      rest of the path, from the child node to the end 
      node 
  return the shortest path 
    """

    local_path = path.copy()
    local_path[0] = local_path[0] + [start]
    #print("New path", local_path[0])
    #print("Current path: ", path)
    if digraph.has_node(start) == False or digraph.has_node(end) == False: 
        raise ValueError("invalid node")
    
    elif start == end:
        #Update global variables appropriately?
        #found path to the destination
        #print("Start = end")
        return local_path
    
    else: 
        #for all children of start construct a path including that node 
        #recursively solve the rest of the path from child node to end node
        for edge in digraph.edges[start]:
            node = edge.get_destination()
            if node not in local_path[0]: 
                edge_dist = edge.get_total_distance()
                outside_dist = edge.get_outdoor_distance()
                local_path[1] = local_path[1] + int(edge_dist)
                local_path[2] = local_path[2] + int(outside_dist)
                if best_path == None or local_path[1] < best_path[1]: # and local_path[2] < max_dist_outdoors):
                    if local_path[2] <= max_dist_outdoors: 
                        new_path = get_best_path(digraph, node, end, local_path, max_dist_outdoors, best_dist,
                      best_path)
                    #print("New path: ", new_path)
                        if new_path != None:
                            #print("Updated best path")
                            best_dist = local_path[1]
                            best_path = new_path
        #print("Best path: ", best_path)        
        return best_path
            
    
    
    


def printPath(path):
    """Assumes path is a list of nodes"""
    result = ''
    for i in range(len(path)):
        result = result + str(path[i])
        if i != len(path) - 1:
            result = result + '->'
    return result 

#directed_dfs(graph, src, dest, 10000, 10000)


# Problem 3c: Implement directed_dfs
def directed_dfs(digraph, start, end, max_total_dist, max_dist_outdoors):
    """
    Finds the shortest path from start to end using a directed depth-first
    search. The total distance traveled on the path must not
    exceed max_total_dist, and the distance spent outdoors on this path must
    not exceed max_dist_outdoors.

    Parameters:
        digraph: Digraph instance
            The graph on which to carry out the search
        start: string
            Building number at which to start
        end: string
            Building number at which to end
        max_total_dist: int
            Maximum total distance on a path
        max_dist_outdoors: int
            Maximum distance spent outdoors on a path

    Returns:
        The shortest-path from start to end, represented by
        a list of building numbers (in strings), [n_1, n_2, ..., n_k],
        where there exists an edge from n_i to n_(i+1) in digraph,
        for all 1 <= i < k

        If there exists no path that satisfies max_total_dist and
        max_dist_outdoors constraints, then raises a ValueError.
    """
    # TODO
    """
    All you are doing in this function is initializing variables, 
    calling your recursive function, and returning the appropriate path.
    """
    # directed_dfs(digraph, start, end, max_total_dist, max_dist_outdoors):
    # get_best_path(digraph, start, end, path, max_dist_outdoors, best_dist,
    # best_path):
    
    # Starting node, total_distance_Traveled, total_distance_outdoors
    path = [[], 0, 0]
    best_dist = None 
    best_path = None 
    start_node = Node(start)
    end_node = Node(end)
    best_path = get_best_path(digraph, start_node, end_node, path, max_dist_outdoors, \
                              best_dist, best_path)
    #Expect best_path to be tuple = {[list of edges], total_distance}
    if best_path == None: # or best_path [1] > max_total_dist: 
        raise ValueError('Too far away, gotta skip class')
    
    print(best_path)
    return best_path[0]

graph = load_map("mit_map.txt")
src = Node(32)
dest = Node(56)
directed_dfs(graph, src, dest, 10000, 0)


# ================================================================
# Begin tests -- you do not need to modify anything below this line
# ================================================================

class Ps2Test(unittest.TestCase):
    LARGE_DIST = 99999

    def setUp(self):
        self.graph = load_map("mit_map.txt")

    def test_load_map_basic(self):
        self.assertTrue(isinstance(self.graph, Digraph))
        self.assertEqual(len(self.graph.nodes), 37)
        all_edges = []
        for _, edges in self.graph.edges.items():
            all_edges += edges  # edges must be dict of node -> list of edges
        all_edges = set(all_edges)
        self.assertEqual(len(all_edges), 129)

    def _print_path_description(self, start, end, total_dist, outdoor_dist):
        constraint = ""
        if outdoor_dist != Ps2Test.LARGE_DIST:
            constraint = "without walking more than {}m outdoors".format(
                outdoor_dist)
        if total_dist != Ps2Test.LARGE_DIST:
            if constraint:
                constraint += ' or {}m total'.format(total_dist)
            else:
                constraint = "without walking more than {}m total".format(
                    total_dist)

        print("------------------------")
        print("Shortest path from Building {} to {} {}".format(
            start, end, constraint))

    def _test_path(self,
                   expectedPath,
                   total_dist=LARGE_DIST,
                   outdoor_dist=LARGE_DIST):
        start, end = expectedPath[0], expectedPath[-1]
        self._print_path_description(start, end, total_dist, outdoor_dist)
        dfsPath = directed_dfs(self.graph, start, end, total_dist, outdoor_dist)
        print("Expected: ", expectedPath)
        print("DFS: ", dfsPath)
        self.assertEqual(expectedPath, dfsPath)

    def _test_impossible_path(self,
                              start,
                              end,
                              total_dist=LARGE_DIST,
                              outdoor_dist=LARGE_DIST):
        self._print_path_description(start, end, total_dist, outdoor_dist)
        with self.assertRaises(ValueError):
            directed_dfs(self.graph, start, end, total_dist, outdoor_dist)

    def test_path_one_step(self):
        self._test_path(expectedPath=['32', '56'])

    def test_path_no_outdoors(self):
        self._test_path(
            expectedPath=['32', '36', '26', '16', '56'], outdoor_dist=0)

    def test_path_multi_step(self):
        self._test_path(expectedPath=['2', '3', '7', '9'])

    def test_path_multi_step_no_outdoors(self):
        self._test_path(
            expectedPath=['2', '4', '10', '13', '9'], outdoor_dist=0)

    def test_path_multi_step2(self):
        self._test_path(expectedPath=['1', '4', '12', '32'])

    def test_path_multi_step_no_outdoors2(self):
        self._test_path(
            expectedPath=['1', '3', '10', '4', '12', '24', '34', '36', '32'],
            outdoor_dist=0)

    def test_impossible_path1(self):
        self._test_impossible_path('8', '50', outdoor_dist=0)

    def test_impossible_path2(self):
        self._test_impossible_path('10', '32', total_dist=100)


if __name__ == "__main__":
    unittest.main()
