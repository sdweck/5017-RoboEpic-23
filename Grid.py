#!/usr/bin/env python
# coding: utf-8

# In[ ]:


class Solution:
    def __init__(self, path = [], score = 0, time = 0):
        self.path = path
        self.score = score
        self.time = time

class Node:
    def __init__(self, x = 0,y = 0, score = 0):
        self.x = x
        self.y = y
        self.score = score
        self.left 
        self.right
    
    #time in seconds (later do distance divided by time)
    def calculate_time(self, other):
    time = 0
    horizontal = other.x - self.x 
    vertical = other.y - self.y
    if horizontal and vertical:
        time = 5
    else:
        time = 2    
    return time

    def index(self):
        for x in range(2):
            for y in range(2):
                curr_node = grid[x][y]
                if y+1 < 3:
                    curr_node.left = grid[x][y+1]
                if x+1 < 3:
                    curr_node.right = grid[x+1][y+1]
            
        
    
    def recurse(s):
        s.score += self.score
        s.path.append(self)
        s.time += calculate_time()
        
        #if left exists, recurse left
        #if right exists, recurse right
        #if left and right don't exist
        #return
        
    
grid = [
    Node(0,0,2),Node(0,1,3),Node(0,2,2),Node(0,3,3),Node(0,4,2),
    Node(1,0,3),Node(1,1,4),Node(1,2,5),Node(1,3,4),Node(1,4,3),
    Node(2,0,2),Node(2,1,5),Node(2,2,2),Node(2,3,5),Node(2,4,2),
    Node(3,0,3),Node(3,1,4),Node(3,2,5),Node(3,3,4),Node(3,4,3),
    Node(4,0,2),Node(4,1,3),Node(4,2,3),Node(4,3,3),Node(4,4,2)
]

