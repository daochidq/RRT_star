# Python File
# Creator : Daoqi Zhang

import math
import random
import numpy as np
import matplotlib.pyplot as plt


class RRT_STAR:

    class Node:
        def __init__(self,pos):
            self.x = pos[0]
            self.y = pos[1]
            self.parent = None
            self.cost = 0

    def __init__(self,start,goal,map_size,goal_radius,extend_dist,
                 obstacle_list,robot_radius,max_iter = 500 ):
        # cost is time
        self.start = self.Node(start)
        self.goal = self.Node(goal)
        self.map_size_min = map_size[0]
        self.map_size_max = map_size[1]
        self.goal_radis = goal_radius
        self.extend_dist = extend_dist
        self.obstacle = obstacle_list
        self.robot_radius = robot_radius
        self.max_iter = max_iter
        self.node_list = []




    def sample(self):
        # returns a state x that is sampled uniformly randomly from the domain
        if random.random() > 0.1:
            x_rand = [random.uniform(self.map_size_min + 0.6,self.map_size_max - 0.6),
                      random.uniform(self.map_size_min + 0.6,self.map_size_max - 0.6)]
        else:
            return self.goal
        return self.Node(x_rand)



    def steer(self,x1, x2):
        '''
        :param x1: x_nearest
        :param x2: x_rand
        :return: x_new
        '''
        # returns the optimal control trajectory of traveling from x1 to x2  and the cost
        d = math.sqrt(math.pow(x1.x - x2.x,2) + math.pow(x1.y - x2.y , 2))
        if d <= self.extend_dist:
            x_new = x2
        else:

            slope = math.atan2((x1.y - x2.y),(x1.x-x2.x))
            x_new_temp = [x1.x + (x2.x - x1.x) * (self.extend_dist / d),
                          x1.y + (x2.y - x1.y) * (self.extend_dist/ d)]
            x_new = self.Node(x_new_temp)

        x_new.parent = x1
        x_new.cost = x1.cost + self.new_cost(x_new, x1)

        return x_new



    def is_in_collision_rect(self,x,y):

        '''
        returns True if state x of the robot is in collision with any of the obstacles
        :param x: node x_new
        :return: bool
        self.obstacle:[[x1,y1,x2,y2]] upper left and lower right
        '''
        for box in self.obstacle:
            if x < box[0]:
                closestPoint_x = box[0]
            elif x > box[2]:
                closestPoint_x = box[2]
            else:
                closestPoint_x = x
            if y > box[1]:
                closestPoint_y = box[1]
            elif y < box[3]:
                closestPoint_y = box[3]
            else:
                closestPoint_y = y

            dist = math.sqrt(math.pow(closestPoint_x-x,2) + math.pow(closestPoint_y-y,2))
            if dist < self.robot_radius:
                return True

        return False

    def check_line_collision(self,x1,x2):
        '''
        check two node connection is collision free or not
        :param x1: x_near
        :param x2:x_new
        :return:
        '''
        # 判断连线与对角线的交点是否在矩形内
        # if x1.x == x2.x:
        #
        #     if self.is_in_collision_rect(x1.x,x1.y) or self.is_in_collision_rect(x2.x,x2.y):
        #         print('part1')
        #         return True
        # else:
        #
        #     s = (x1.y - x2.y) / (x1.x - x2.x)
        #     b = x1.y - s*x1.x
        #     for box in self.obstacle:
        #
        #         s1 = (box[1]-box[3]) / (box[0]- box[2])
        #         b1 = box[1] - s1*box[0]
        #         s2 = (box[1]-box[3]) / (box[2] - box[0])
        #         b2 = box[1] - s2*box[2]
        #
        #         its1_x = (b-b1)/(s1-s)
        #         its1_y = (b-b1)/(s1-s) * s + b
        #
        #         its2_x = (b - b2) / (s2 - s)
        #         its2_y = (b - b2) / (s2 - s) * s + b
        #
        #         if self.is_in_collision_rect(its1_x,its1_y) or self.is_in_collision_rect(its2_x,its2_y):
        #             print('here',x1.x,x1.y,x2.x,x2.y)
        #             return True
        # return False

        for i in range(70):
            x = x1.x + (i / 70) * (x2.x - x1.x)
            y = x1.y + (i / 70) * (x2.y - x1.y)

            # if self.is_in_collision_rect(x, y):
            #     return True
            if (3.4 < x < 5.6) and (-4.6 < y < 10):
                return True

            elif (-6.6 < x < 0.6) and (-5.6 < y < -3.4):
                return True

        return False



    def nearest(self,x):
        '''
        finds a node in the tree that is closest to the state x (closest in what metric?)
        :param x: x_rand
        :return: the nearesy node
        '''
        dist = [math.pow(node.x - x.x,2)+math.pow(node.y - x.y,2) for node in self.node_list]
        index = dist.index(min(dist))

        return self.node_list[index]


    def near_nodes(self,x_new):
        '''
        find all near node in a circle
        :param node_list:
        :param x_new:
        :return: list of nodes that is in the circle
        '''
        n = len(self.node_list) + 1
        r = 15*(math.log(n)/n)**(1/2)
        dist = [math.sqrt(math.pow(node.x - x_new.x,2)+math.pow(node.y - x_new.y,2)) for node in self.node_list]
        nodes = [self.node_list[dist.index(i)] for i in dist if i <= r]
        # print(nodes)
        return nodes

    def choose_parent(self,x_new,nodes):
        '''

        :param x_new: new node
        :param nodes: list of nearby node
        :return:
        '''
        cost = []
        if nodes == []:
            return None

        for node in nodes:
            temp_node = self.steer(node,x_new)
            if self.check_line_collision(temp_node,node) or self.is_in_collision_rect(temp_node.x,temp_node.y):
                cost.append(float('inf'))
            else:
                cost.append(node.cost + self.new_cost(temp_node,node))
        min_cost = min(cost)
        if min_cost == float('inf'):
            return None
        index = cost.index(min_cost)
        new_parent = nodes[index]

        return new_parent

    def connect(self,x_new,parent):
        # add state x to the tree
        # cost of state x = cost of the parent + cost of steer(parent, x)
        self.node_list.append(x_new)
        x_new.parent = parent
        x_new.cost = parent.cost + self.new_cost(x_new,parent)

    def new_cost(self,x1,x2):

        return math.sqrt(math.pow(x1.x - x2.x ,2)+ math.pow(x1.y - x2.y,2))


    def rewire(self,x_new,nodes):
        # rewire all nodes in the tree within the O(gamma (log n/n)ˆ{1/d}} ball # near the state x,
        # update the costs of all rewired neighbors
        for node in nodes:
            temp_cost = x_new.cost + self.new_cost(x_new,node)
            if temp_cost < node.cost:
                if self.check_line_collision(node,x_new):
                    continue
                node.cost = temp_cost
                node.parent = x_new

    def is_goal(self,new_node):

        return self.new_cost(new_node,self.goal) <= self.goal_radis

    def find_final_path(self,x_new):
        path = []
        while x_new.parent != None:
            path.append([x_new.x,x_new.y])
            x_new = x_new.parent
        path.append([x_new.x,x_new.y])

        return path

    def plot_graph(self):
        plt.plot(self.start.x, self.start.y, "xr")
        # plt.plot(self.goal.x, self.goal.y, "xr")
        plt.scatter(self.goal.x, self.goal.y, s=120, facecolors='blue')
        RECT = plt.Rectangle((4,-4),1,13,fill=True,color='gray')
        RECT1 = plt.Rectangle((-6,-5),6,1,fill=True,color='gray')
        plt.gca().add_patch(RECT)
        plt.gca().add_patch(RECT1)
        plt.axis("equal")
        plt.axis([-10, 10, -10, 10])
        for node in self.node_list:
            p = node.parent
            if p:
                plt.plot([node.x,p.x],[node.y,p.y],'-g')


    def plot_final_path(self,path):
        plt.plot([x for (x,y) in path],[y for (x,y) in path],'-r')

    def planning(self):
        path = []
        cost_iter = []
        best_cost = float('inf')
        for i in range(10000):
            sample = self.sample()
            x_nearsest = self.nearest(sample)
            x_new = self.steer(x_nearsest, sample)

            if self.check_line_collision(x_new,x_nearsest):
                # print('collision')
                continue
            else:
                # self.node_list.append(x_new)
                # print(i)
                nearby_nodes = self.near_nodes(x_new)
                new_parent = self.choose_parent(x_new,nearby_nodes)
                if not new_parent:
                    print('no parent')
                    continue
                else:

                    self.connect(x_new,new_parent)
                    self.rewire(x_new,nearby_nodes)

                if self.is_goal(x_new):
                    print('Goal found')
                    if x_new.cost < best_cost:
                        best_cost = x_new.cost
                        path = self.find_final_path(x_new)
                        cost_iter.append((x_new.cost,i))

        self.plot_graph()
        self.plot_final_path(path)
        plt.figure()
        plt.plot([y for (x,y) in cost_iter],[x for (x,y) in cost_iter],'-b')
        plt.title("Best path cost vs. iteration")
        plt.xlabel("Iteration")
        plt.ylabel('Best path cost')
        plt.show()
        print('best cost: ', best_cost)
        print('best path:', path)



def main():
    start = [0,0]
    goal = [8,8]
    map_size = [-10,10]
    ob = [[4,9,5,-4],[-6,-4,0,-5]]
    rrt = RRT_STAR(start,goal,map_size,0.5,0.5,ob,robot_radius = 0.6,max_iter = 10000 )
    rrt.node_list.append(rrt.start)
    rrt.planning()
    # print(rrt.is_in_collision_rect(rrt.Node([4.87,-3.9])))
    # x1 = rrt.Node([6.2357062667636285, -2.385969148052582])
    # x2 = rrt.Node([5.984590039423073 ,-2.414356593062576])
    # print(rrt.check_line_collision(x1,x2))

if __name__ == '__main__':
    main()