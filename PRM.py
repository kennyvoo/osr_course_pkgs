import numpy as np
import argparse
import os
from dijkstra import dijkstra
from shapely.geometry import Polygon,LineString
from sklearn.neighbors import NearestNeighbors

import pylab as pl
import sys
from dijkstra import dijkstra
sys.path.append('osr_examples/scripts/')
import environment_2d


class PRM_planner:
  """
  A Probabilistic Roadmap Path Planning by refencing to the pseudocode at
  http://www.cs.columbia.edu/~allen/F15/NOTES/Probabilisticpath.pdf

  """

  def __init__(self,env,init,goal):
    self.env = env
    self.init = init
    self.goal = goal
    self.local_planner = dijkstra()
    self.obs_polygons = []
    self.edges = set()
    self.vertices = set()
    self.solution = None

  def construct_graph(self,n_sample = 200, n_neighbors = 5, max_radius = 1):
    """
    Randomly sample n_sample of vertices, and form collision-free edges
    with vertices around max_radius 

    n_sample       - number of vertices to sample from map
    n_neighbors    - number of neighbors to connect between vertices
    max_radius     - max search space / maximum length of edges

    """
    self.edges = set()
    self.vertices = set()


    # randomly sample vertices 0.5 away from the map border
    # vertices=np.round(np.random.rand(n_sample,2)* [self.env.size_x-1,self.env.size_y-1]+[0.5,0.5],2)
    vertices = np.round(np.random.rand(n_sample,2)* [self.env.size_x,self.env.size_y],2)

    # retrieve obstacle from map
    self.get_obstacles_polygons()

    # Get collision free vertices
    free_vertices = []
    for x,y in vertices:
        if not self.env.check_collision(x,y):
            free_vertices.append((x,y))

    # form edges with vertices 
    self.form_edges(free_vertices,n_neighbors,max_radius)


  def form_edges(self,free_vertices, n_neighbors,max_rad):
    """
    form up to n_neighbors of edges with the vertices in the max_rad space 

    free_vertices - number of vertices to sample from map
    n_neighbors   - number of neighbors to connect between vertices
    max_rad      -  max search space/ maximum length of edges
    no_min         - disable minimum length restriction

    """
    # +1 to have 5 edges excluding itself
    knn = NearestNeighbors(n_neighbors=n_neighbors+1,radius=max_rad).fit(free_vertices)
    distance,indices = knn.kneighbors(free_vertices)
    
    # arr[0] is the vertex itself
    for arr,dists in zip(indices,distance):
        
        self.vertices.add(free_vertices[0])
        # find the nearest neighbour excluding itself
        for index,dist in zip(arr[1:], dists):
            #retrieve the 2 connected vertices
            v1,v2 = free_vertices[arr[0]],free_vertices[index]
            
            # check if they intersect with the obstacles
            if not self.check_collision([v1,v2]):
                
            # # skip edges shorter than min_dist
                # if np.sqrt((v2[0] - v1[0])**2 + (v2[1] - v1[1])**2)<0.5:
                #     continue
                self.vertices.add(free_vertices[index])

                #if v1>v2:
                self.edges.add((v1 ,v2,round(dist,3)))
                # else:
                #     self.edges.add((v2, v1,round(dist,3)))

    self.edges = list(self.edges)
    self.vertices = list(self.vertices)

  def get_obstacles_polygons(self):
    """
    Form polygons of the obstacles
    """
    for obs in self.env.obs:
        self.obs_polygons.append([(obs.x0,obs.y0),(obs.x1,obs.y1),(obs.x2,obs.y2)])

  def check_collision(self,line):
    """
    Check if there's collision between edges and obstacles in the map
    
    line - 2 points that form a line [pt1,pt2]

    """
    for obs_polygon in self.obs_polygons:
        p1 = Polygon(obs_polygon)
        p2 = LineString(line)
        if p1.intersects(p2):
            return True # collision
    return False #no collision

  def connect_nearest_vertex(self,point,n_neighbors=20,radius=5,front=True):
    """
    find nearest points to point and add it to the edges list if exist

    point - any point in the map
    n_neighbors   - number of neighbors to connect between vertices
    radius  - maximum space to search for / maximum length of edges
    """
    knn = NearestNeighbors(n_neighbors=n_neighbors,radius=radius).fit(self.vertices)
    # distance is already sorted
    distance,indices = knn.kneighbors([point])

    connected=False
    for index,dist in zip(indices[0], distance[0]):
        v1,v2 = point,self.vertices[index]
        if not self.check_collision([v1,v2]):
            connected = True
            if front:
              self.edges.append((v1 ,v2,round(dist,2)))
            else:
              self.edges.append((v2 ,v1,round(dist,2)))

    return connected

  def find_path(self):
      """
      Connect starting point and goal to the nearest vertex. 
      Followed by using local planner to find the shortest path

      """
      # if can't connect means no solution found
      if not self.connect_nearest_vertex(self.init) or not self.connect_nearest_vertex(self.goal,front=False):
          print("No solution : No nearby vertex to connect")
          return False

      # find nearest path
      self.local_planner.set_edges(self.edges)
      # print(self.edges)
      returned_path, returned_distance = self.local_planner.shortest_path(self.init, self.goal)
      # print(returned_path)
      if len(returned_path)>1:
        print(returned_path) 
        self.solution = returned_path
      else:
        print("No solution")
        return False

      return True #path found

  def optimize_path(self):
      """
      Using brute force method to optimize the path
      """
      left = 0
      right = len(self.solution)-1
      optimized_path=[]
      while(left != right):
        for i in range(right-left):
            if left == right-i:
              optimized_path.append(left)
              break
            if not self.check_collision([self.solution[left],self.solution[right-i]]):
              optimized_path.append(self.solution[left])
              optimized_path.append(self.solution[right-i])
              left = right-i
              break
        left += 1
        if optimized_path[-1] == self.solution[-1]:
          break

      return optimized_path




if __name__ == "__main__":
    parser= argparse.ArgumentParser(description="PRM planner")
    parser.add_argument("--seed",help="set seed",default=4)
    parser.add_argument("-s","--n_sample",help="number of vertex to start with",default=1000)
    parser.add_argument("-n","--n_neighbors",help="no of nearest vertex to connect",default=10)
    parser.add_argument("-r","--max_radius",help="maximum search space",default=10)

    parser.add_argument("--map_width",help="width of map",default=10)
    parser.add_argument("--map_height",help="height of map", default=6)
    parser.add_argument("--n_obstacle",help="number of obstacle",default=5)
    
    parser.add_argument("--output_image_path",help="output folder of map",default="result")
  
    args=parser.parse_args()

    if not os.path.exists(args.output_image_path):
          os.mkdir(args.output_image_path)

    # pl.ion()
    np.random.seed(int(args.seed))
    env = environment_2d.Environment(args.map_width, args.map_height, args.n_obstacle)
    pl.clf()
    env.plot()
    q = env.random_query()
    if q is not None:
      x_start, y_start, x_goal, y_goal = q
      env.plot_query(x_start, y_start, x_goal, y_goal)
    # save initial map
    pl.savefig(f"{args.output_image_path}/1_initial_map.png")
    

    
    ## Path planning
    prm = PRM_planner(env,(round(x_start,2),round(y_start,2)),(round(x_goal,2),round(y_goal,2)))
    prm.construct_graph(n_sample=int(args.n_sample),n_neighbors=int(args.n_neighbors),max_radius=int(args.max_radius))
    
    
    for edge in prm.edges:
        x = [edge[0][0],edge[1][0]]
        y = [edge[0][1],edge[1][1]]
        pl.plot(x,y,color='blue',linewidth=0.5)

    pl.savefig(f"{args.output_image_path}/2_map_with_edges.png")


    #if no solution, exit 
    if not prm.find_path():   
      sys.exit(0)
 

    # save map with solution path 
    x = np.array(prm.solution)[:,0]
    y = np.array(prm.solution)[:,1]
    pl.plot(x[1:-2],y[1:-2],color='red',linewidth=3)
    pl.plot(x[:2],y[:2],color='yellow',linewidth=3)
    pl.plot(x[-2:],y[-2:],color='yellow',linewidth=3)
    pl.title("Solution Found")

    pl.savefig(f"{args.output_image_path}/3_solution_path.png")
  #  pl.show(block = True)

    
    #optimized_path
    optimized_path= prm.optimize_path()
    pl.clf()
    env.plot()
    env.plot_query(x_start, y_start, x_goal, y_goal)
    x= np.array(optimized_path)[:,0]
    y= np.array(optimized_path)[:,1]
    pl.plot(x,y,color='yellow',linewidth=3)
    pl.title("Optimized Solution")
    pl.savefig(f"{args.output_image_path}/4_optimized_path.png")
