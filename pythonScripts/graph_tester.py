from graph_tool import Graph
from graph_tool.draw import graph_draw
from graph_tool.search import astar_search
from pathlib import Path

# example graph creation
g = Graph()
v1 = g.add_vertex()
v2 = g.add_vertex()
e = g.add_edge(v1, v2)