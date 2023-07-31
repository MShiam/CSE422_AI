#!/usr/bin/env python
# coding: utf-8

# In[7]:


file=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\input1.txt","r")
#file is to read the whole text 
file2=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\output1.txt","w")
file3=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\input1.txt","r")
#this is to read by lines as i have read the whole text file earlier so i have made another file named file3 to readline
file4=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\output1.txt","w")
k=file.read()
c=1
store=[]
H={}
#H is the dictionary where i have saved huristic values
#store basically the big list where there are several sub lists
#in the for loop below basically i determined number of lines there are in the text file so i have taken counter as c to count
for i in k:
    if i=="\n":
        c+=1
for i in range(c):
    k2=file3.readline().split()
    #here i have made list for every line
    store.append(k2)
for i in store:
    for j in range(len(i)):
        if j==0:
            H[i[j]]=int(i[1])

#saved huristic values in H
print(store)


# In[2]:



import heapq

file=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\input1.txt","r")
file2=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\output1.txt","w")
heuristic_d={}
graph={}
for line in file:
   d={}
   k=line.strip().split()
   heuristic_d[k[0]]=k[1]
   for i  in range(2,len(k)-1,2):
      if  i!=0 and i!=1:
        d[k[i]]=k[i+1]
        graph[k[0]]=d
print(graph)
print(heuristic_d)


# In[58]:


from collections import defaultdict
import heapq
file=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\input1.txt","r") 
file2=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\output1.txt","w")
def build_graph(file):
    k=file.readlines()
    # Function to build the graph from the input file
    graph = defaultdict(dict)
    for line in k:
            # Read each line from the file
            cities, distance = line.strip().split(' ')
            city1, city2 = cities.split('-')
            distance = int(distance)
            # Store the distances between cities in the graph
            graph[city1][city2] = distance
            graph[city2][city1] = distance
    return graph
graph = build_graph(file)
def heuristic(city, target, graph):
    # Function to calculate the heuristic value between cities
    return graph[city][target]

def astar_search(graph, start, target):
    # A* search algorithm implementation
    open_list = [(0, start)]
    visited = set()

    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0

    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, target, graph)

    while open_list:
        # Continue searching until all cities are explored
        current_distance, current_city = heapq.heappop(open_list)

        if current_city == target:
            # If the target city is reached, return the minimum distance
            return g_score[current_city]

        visited.add(current_city)

        for neighbor, distance in graph[current_city].items():
            if neighbor in visited:
                continue

            tentative_g_score = g_score[current_city] + distance

            if tentative_g_score < g_score[neighbor]:
                # Update the g_score and f_score for the neighbor
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, target, graph)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return float('inf')  # No path found

 # Replace with the path to your input file

start_city = 'Arad'
target_city = 'Bucharest'

distance = astar_search(graph, start_city, target_city)
if distance != float('inf'):
    print(f"The minimum distance between {start_city} and {target_city} is {distance} units.")
else:
    print(f"No path found between {start_city} and {target_city}.")


# In[59]:


from collections import defaultdict
import heapq

file = open(r"C:\Users\MAS COMPUTER\Desktop\lab_1_221\input1.txt", "r")
file2 = open(r"C:\Users\MAS COMPUTER\Desktop\lab_1_221\output1.txt", "w")

def build_graph(file):
    # Function to build the graph from the input file
    graph = defaultdict(dict)
    for line in file:
        # Read each line from the file
        cities, distance = line.strip().split(' ')
        city1, city2 = cities.split('-')
        distance = int(distance)
        # Store the distances between cities in the graph
        graph[city1][city2] = distance
        graph[city2][city1] = distance
    return graph



# In[60]:


from collections import defaultdict
import heapq

file = open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\input1.txt", "r")
file2 = open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\output1.txt", "w")

def build_graph(file):
    k = file.readlines()
    # Function to build the graph from the input file
    graph = defaultdict(dict)
    for line in k:
        # Read each line from the file
        cities, distance = line.strip().split(' ')
        city1, city2 = cities.split('-')
        distance = int(distance)
        # Store the distances between cities in the graph
        graph[city1][city2] = distance
        graph[city2][city1] = distance
    return graph

g


# In[63]:


import heapq
from collections import defaultdict

def heuristic(node, goal):
    # Heuristic function (example: Euclidean distance)
    x1, y1 = node  # Assuming the node is a tuple with (x, y) coordinates
    x2, y2 = goal
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

def a_star_search(graph, start, goal):
    # A* search algorithm implementation
    open_list = [(0, start)]
    visited = set()

    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0

    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, goal)

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        if current_node == goal:
            # Goal reached, return the path
            path = []
            while current_node != start:
                path.append(current_node)
                current_node = graph[current_node]['parent']
            path.append(start)
            path.reverse()
            return path

        visited.add(current_node)

        for neighbor, distance in graph[current_node].items():
            if neighbor in visited:
                continue

            tentative_g_score = g_score[current_node] + distance

            if tentative_g_score < g_score[neighbor]:
                # Update the g_score and f_score for the neighbor
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))
                graph[neighbor]['parent'] = current_node

    return None  # No path found

# Example graph representation (dictionary of dictionaries)
graph = {
    'A': {'B': 5, 'C': 3},
    'B': {'A': 5, 'D': 2},
    'C': {'A': 3, 'D': 6, 'E': 4},
    'D': {'B': 2, 'C': 6, 'E': 1},
    'E': {'C': 4, 'D': 1, 'F': 8},
    'F': {'E': 8}
}

start_node = (0, 0)  # Example start node coordinates
goal_node = (3, 4)  # Example goal node coordinates

path = a_star_search(graph, start_node, goal_node)

if path is not None:
    print(f"Path from {start_node} to {goal_node}: {' -> '.join(map(str, path))}")
else:
    print(f"No path found from {start_node} to {goal_node}.")


# In[62]:


import heapq
from collections import defaultdict

def heuristic(node, goal):
    # Heuristic function (example: Euclidean distance)
    x1, y1 = node
    x2, y2 = goal
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

def a_star_search(graph, start, goal):
    # A* search algorithm implementation
    open_list = [(0, start)]
    visited = set()

    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0

    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, goal)

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        if current_node == goal:
            # Goal reached, return the path
            path = []
            while current_node != start:
                path.append(current_node)
                current_node = graph[current_node]['parent']
            path.append(start)
            path.reverse()
            return path

        visited.add(current_node)

        for neighbor, distance in graph[current_node].items():
            if neighbor in visited:
                continue

            tentative_g_score = g_score[current_node] + distance

            if tentative_g_score < g_score[neighbor]:
                # Update the g_score and f_score for the neighbor
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))
                graph[neighbor]['parent'] = current_node

    return None  # No path found

# Example graph representation (dictionary of dictionaries)
graph = {
    'A': {'B': 5, 'C': 3},
    'B': {'A': 5, 'D': 2},
    'C': {'A': 3, 'D': 6, 'E': 4},
    'D': {'B': 2, 'C': 6, 'E': 1},
    'E': {'C': 4, 'D': 1, 'F': 8},
    'F': {'E': 8}
}

start_node = 'A'
goal_node = 'F'

path = a_star_search(graph, start_node, goal_node)

if path is not None:
    print(f"Path from {start_node} to {goal_node}: {' -> '.join(path)}")
else:
    print(f"No path found from {start_node} to {goal_node}.")


# In[ ]:





# In[6]:


import heapq

file=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\input1.txt","r")
file2=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\output1.txt","w")
heuristic_d={}
graph={}
for line in file:
   d={}
   k=line.strip().split()
   heuristic_d[k[0]]=k[1]
   for i  in range(2,len(k)-1,2):
      if  i!=0 and i!=1:
        d[k[i]]=k[i+1]
        graph[k[0]]=d
def a_srch(start,goal,graph,heuristic_d):
    queue = [(0, start)]
    t_cost = {city: float('inf') for city in graph}
    t_cost[start] = 0
    parent = {start: None}
    while queue:
        c_cost, c_city = heapq.heappop(queue)
        if c_city == goal:
            path = []
            while c_city:
                path.append(c_city)
                c_city = parent[c_city]
            path.reverse()
            return c_cost,path
        for n,dis in graph[c_city].items():
            n_cost = int(t_cost[c_city]) + int(dis)
            e_cost = n_cost + int(heuristic_d[n])
            if n_cost < t_cost[n]:
                t_cost[n] = n_cost
                parent[n] = c_city
                heapq.heappush(queue, (e_cost,n))
    return None
start_city = input("Enter the starting city: ")
goal_city = input("Enter the goal city: ")
cost,path = a_srch(start_city, goal_city,graph,heuristic_d)
file2.write(f"Total distance {cost}")
f_p=''
for i in path:
  if i==goal_city:
    f_p+=i
  else:
    f_p+=i+"->"
file2.write(f_p)


# In[9]:


import math

import heapq

file=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\input1.txt","r")
file2=open("C:\\Users\\MAS COMPUTER\\Desktop\\lab_1_221\output1.txt","w")
heuristic={}
d={}
for line in file:
   di={}
   k=line.strip().split()
   heuristic[k[0]]=k[1]
   for i  in range(2,len(k)-1,2):
      if  i!=0 and i!=1:
        di[k[i]]=k[i+1]
        d[k[0]]=di


ndn={}
parent={}
visited=[]
def astar(d,src,destination,heuristic):
    ndn.update({src:0})
    key=list(ndn.keys())
    valu=list(ndn.values())
    while key:
        skey=key[valu.index(min(valu))]
        visited.append(skey)
        for i in d[skey]:
            if i[0] not in visited:
                now=int(valu[valu.index(min(valu))])+int(i[1])+int(heuristic[i[0]])

            try:
                if now<valu[key.index(i[0])]:
                    matha=now-heuristic[i[0]]
                    valu[key.index(i[0])]=matha
                    parent.update({i[0]:skey})
            except:
                pass

        a_1=key.pop(valu.index(min(valu)))
        b_1=valu.pop(valu.index(min(valu)))
        ndn.update({a_1:b_1})

src='Arad'
destination='Bucharest'
parent={str(i):None for i,j in d.items()}
ndn={str(i):math.inf for i,j in d.items()}
astar(d,src,destination,heuristic)
print("total distance:",ndn.get(destination))
print(f'the path is:')
fn=[destination]
while True:
    i=parent.get(destination)
    if i==None:
        break
    else:
        fn.append(i)
        destination=i
for i in range(len(fn)-1,-1,-1):
    print(fn[i]+" ")


# In[5]:


a=[1,2,3,4,5]
a.append(12)
print(a)


# In[ ]:




