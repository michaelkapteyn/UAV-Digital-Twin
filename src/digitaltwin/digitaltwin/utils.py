import json
from collections import OrderedDict
import itertools

def read_json_file(json_file):
    f = open(json_file, 'r')
    ref_json = json.load(f,object_pairs_hook=OrderedDict)
    f.close()
    return ref_json

def write_json_file(filepath, data):
    with open(filepath, "w") as f:
        f.write(json.dumps(data, sort_keys=True, indent=4, ensure_ascii=False))
    f.close()

def prettyprint(dict):
    print(json.dumps(dict, indent=1))

def graph_to_string(graph):
    """Convert pomegranate graph to dot string using pygraphviz.
    Returns
    -------
    string of graph in dot format
    """

    G = pygraphviz.AGraph(directed=True)
    for state in graph.states:
        G.add_node(state.name, color='red')

    for parent, child in graph.edges:
        G.add_edge(parent.name, child.name)

    return G.to_string()

def flatten_list(states):
    return list(itertools.product(*states))
