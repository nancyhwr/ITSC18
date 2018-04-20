
## This is the implementation for IA-Dijskatra
## two function: ia_dijskatra: for group nodes under the threshold
##               merge_groups: merge two groups 

class Graph:

  def __init__(self, nodes, edges, stream_rates):
    self.nodes = nodes
    self.edges = edges
    self.stream_rates = stream_rates


  def add_node(self, value):
    self.nodes.add(value)

  def remove_node(self, nodes):
    for node in nodes:
      self.nodes.remove(node)
      self.stream_rates.pop(node)


  def add_edge(self, from_node, to_node, distance):
    self.edges[from_node].append(to_node)
    self.edges[to_node].append(from_node)
    self.distances[(from_node, to_node)] = distance



def ia_dijskatra(graph, initial, stream_rates, threshold):

  total = stream_rates[initial]
  visited = {initial: stream_rates[initial]}
  c_node = initial
  

  while total <= threshold:
    neighbors = {}

    if len(graph.edges[c_node]) == 0:
      print('singleton:', c_node)

      return ([c_node], total)

    else:

      for p in graph.edges[c_node]:
        if (p in graph.nodes) and (p not in list(visited.keys())) and visited[c_node]+stream_rates[p] <= threshold:
          neighbors[p] = visited[c_node]+stream_rates[p]

      if len(list(neighbors.keys())) > 0:
        max_node = max(neighbors, key = neighbors.get)
        visited[max_node] = neighbors[max_node]
        c_node = max_node
        total = visited[c_node]
      else:

        return (list(visited.keys()), max(list(visited.values())))

    return (list(visited.keys()), max(list(visited.values())))


def merge_groups(groups, threshold):
  total = 0
  new_group = []
  cum_group = groups[0]
  i = 0
  for i in range(1, len(groups)):
    if cum_group[1] + groups[i][1] < threshold:
      cum_group = (cum_group[0]+groups[i][0], cum_group[1]+groups[i][1])
    else:
      new_group.append(cum_group)
      cum_group = groups[i]
  new_group.append(cum_group)
  return new_group



def coalition(graph, stream_rates,threshold):
  groups = []

  while len(graph.nodes)>0:
    initial = max(stream_rates, key=stream_rates.get)
    group = ia_dijskatra(graph, initial, stream_rates, threshold)
    groups.append(group)
    graph.remove_node(group[0][i] for i in range(len(group[0])))
  return merge_groups(groups, threshold)













