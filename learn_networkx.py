import networkx as nx
import matplotlib.pyplot as plt


G = nx.Graph()

G.add_node(1)
G.add_nodes_from([2,3])
G.add_nodes_from([
    (4,{"color":"red"}),
    (5,{"color":"green"})
])
H = nx.path_graph(10)
#G.add_node(H)

G.add_edge(1,2)
e = (2,3)
G.add_edge(*e)

G.add_edges_from([(3,4),(4,5),(5,2)])



subax1 = plt.subplot(221)
nx.draw(G, with_labels=True, font_weight='bold')

#print(G)


DG = nx.DiGraph()
DG.add_edge(2,1)
DG.add_edge(1,3)
DG.add_edge(2,4)
DG.add_edge(1,2)

subax2 = plt.subplot(222)
nx.draw(DG, with_labels=True, font_weight='bold')

#print(DG.edges)


FG = nx.Graph()
FG.add_weighted_edges_from([(1,2,0.125),(1,3,0.75),(2,4,1.2),(3,4,0.375)])
#print(FG.adj.items)
subax3 = plt.subplot(223)
nx.draw(FG, with_labels=True, font_weight='bold')
plt.savefig("path.png")
plt.show()


for n,nbrs in FG.adj.items():
    for nbr, eattr in nbrs.items():
        wt = eattr['weight']
        if wt < 0.5:
            print(f"{n},{nbr},{wt:.3}")
            #print("%i,%i,%.3f"  %(n,nbr,wt))


print(FG.edges.data)

for (u,v,wt) in FG.edges.data('weight'):
    if wt < 0.5:
        print(f"{u},{v},{wt:.3}")






#G = nx.path_graph(5, create_using = nx.DiGraph())