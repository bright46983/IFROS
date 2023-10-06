import numpy as np

class WaveFrontAlgo():
    def __init__(self) -> None:
        pass

    def isOutofBound(self,index, map):
        try: #Check out of bound for exceeded index
            if index[0] < 0 or index[1] < 0 or index[0]>=map.shape[0] or index[1]>=map.shape[1]: #Check out of bound for -1 index
                return False
            else:
                return True
        except:
                return False
            
    def isValid(self,index, map):
        if self.isOutofBound(index,map):
            if map[index[0],index[1]] == 1: # Check Obstacle
                return False
            else: # apart from value 1 is valid
                return True
        else:
            return False
                        
    def isRightPath(self,origin,index, map):
        try:
                if index[0] < 0 or index[1] < 0:
                    return False
                if map[index[0],index[1]] < origin and map[index[0],index[1]] != 1:
                    return True
                else:
                    return False
           
        except:
            return False
        
    

    def wavefront_planner_connect_4(self, grid_map:np.array, goal:list):
        """
        Create attraction function with 4-connectivity Wavefront 
        """
        motions = [[-1,0],[1,0],[0,-1],[0,1]] # Up, Down, Left, Right
        
        goal_value = 2 
        goal_row = goal[0]
        goal_col = goal[1] 
        queue = [goal]
        
        # manually assign goal value in map 
        grid_map[goal_row,goal_col] = goal_value 
        #print(grid_map)

        # Start Wavefront loop 
        while queue:
            goal_value = goal_value + 1
            new_queue = []
            
            for p in queue:
                for m in motions:
                    index = [p[0]+m[0],p[1]+m[1]] 
                   # print(index)
                    if self.isValid(index, grid_map):
                        old_goal_value = grid_map[index[0],index[1]]
                        new_goal_value = goal_value 
                        if old_goal_value == 0: # Not explored space just fill in first
                            grid_map[index[0],index[1]] =  new_goal_value
                            new_queue.append(index)
                        elif old_goal_value > new_goal_value: # New value is a shorter path 
                            grid_map[index[0],index[1]] =  new_goal_value
                            new_queue.append(index)
                    
            queue = new_queue
            #print(grid_map)

        final_map = grid_map
        return final_map
    
    def wavefront_planner_connect_8(self, grid_map:np.array, goal:list):
        """
        Create attraction function with 8-connectivity Wavefront 
        """
        motions = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]] # Up, Down, Left, Right, All Diagonals
        
        goal_value = 2 
        goal_row = goal[0]
        goal_col = goal[1] 
        queue = [goal]
        
        # manually assign goal value in map 
        grid_map[goal_row,goal_col] = goal_value 
        #print(grid_map)

        # Start Wavefront loop 
        while queue:
            goal_value = goal_value + 1
            new_queue = []
            
            for p in queue:
                for m in motions:
                    index = [p[0]+m[0],p[1]+m[1]] 
                   
                    if self.isValid(index, grid_map):
                        old_goal_value = grid_map[index[0],index[1]]
                        new_goal_value = goal_value 
                        if old_goal_value == 0: # Not explored space just fill in first
                            grid_map[index[0],index[1]] =  new_goal_value
                            new_queue.append(index)
                        elif old_goal_value > new_goal_value: # New value is a shorter path 
                            grid_map[index[0],index[1]] =  new_goal_value
                            new_queue.append(index)

            queue = new_queue
            #print(grid_map)

        final_map = grid_map
        return final_map
    
    def wavefront_planner_euclidean(self, grid_map:np.array, goal:list):
        """
        Create attraction function with 8-connectivity Wavefront 
        """
        motions = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]] # Up, Down, Left, Right, All Diagonals
        
        goal_value = 2 
        goal_row = goal[0]
        goal_col = goal[1] 
        queue = [goal]
        
        # manually assign goal value in map 
        grid_map[goal_row,goal_col] = goal_value 
        #print(grid_map)

        # Start Wavefront loop 
        while queue:
            new_queue = []
            
            for p in queue:
                goal_value = grid_map[p[0],p[1]]
                for m in motions:
                    index = [p[0]+m[0],p[1]+m[1]] 
                   # print(index)
                    if self.isValid(index, grid_map):
                        old_goal_value = grid_map[index[0],index[1]]
                        new_goal_value = goal_value + ((index[0] - p[0])**2+(index[1] - p[1])**2)**0.5
                        if old_goal_value == 0: # Not explored space just fill in first
                            grid_map[index[0],index[1]] =  new_goal_value
                            new_queue.append(index)
                        elif old_goal_value > new_goal_value: # New value is a shorter path 
                            grid_map[index[0],index[1]] =  new_goal_value
                            new_queue.append(index)
                        
                        
                    
            queue = new_queue
            #print(grid_map)

        final_map = grid_map
        return final_map
    
    def find_the_path_4(self,map:np.array, start:list):
        #Check the neighbor 
        motions = [[-1,0],[1,0],[0,-1],[0,1]] # Up, Down, Left, Right

        queue = [start[0] , start[1]]
        path = [start]
        while queue:
            new_queue = []
            origin = map[queue[0],queue[1]]
            
            value_list = []
            index_list = []
            for m in motions:
                index = [queue[0]+m[0],queue[1]+m[1]] 
                
                if self.isOutofBound(index, map):
                    value = map[index[0],index[1]]
                    if value < origin:
                        value_list.append(value)
                        index_list.append(index)
            if index_list:
                selected_index = index_list[np.argmin(value_list)] #choose the lowest value as a path
                new_queue = selected_index
                path.append(selected_index)
                           
            queue = new_queue
        return path
        
    def find_the_path_8(self,map:np.array, start:list):
        #Check the neighbor 
        motions = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]] # Up, Down, Left, Right, All Diagonals

        queue = [start[0] , start[1]]
        path = [start]
        while queue:
            new_queue = []
            origin = map[queue[0],queue[1]]
            
            value_list = []
            index_list = []
            for m in motions:
                index = [queue[0]+m[0],queue[1]+m[1]] 
                
                if self.isOutofBound(index, map):
                    value = map[index[0],index[1]]
                    if value < origin:
                        value_list.append(value)
                        index_list.append(index)
            if index_list:
                selected_index = index_list[np.argmin(value_list)] #choose the lowest value as a path
                new_queue = selected_index
                path.append(selected_index)
                           
            queue = new_queue
        return path
    
    def find_lcoal_minimum(self,map:np.array):
        #Check the neighbor 
        motions = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]] # Up, Down, Left, Right, All Diagonals
        local_min_list = []
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                value = map[i, j] 
                if value < map.max() -0.00001: 
                    for m in motions:
                        is_local_min = True
                        n_index = [i+m[0],j+m[1]] 
                        if self.isOutofBound(n_index, map):
                            n_value = map[n_index[0],n_index[1]]
                            if n_value < map.max()-0.00001:  #not obstacle
                                if n_value < value  :
                                    is_local_min = False
                                    break
                    
                    if is_local_min:
                        local_min_list.append([i,j])

        return local_min_list                        

                            
def testing_array():
    # Small Array for Testing
    array = np.zeros((14, 20), dtype=int)
    array[:,[0,-1]] = 1   # We add 1's 
    array[[0,-1],:] = 1   # as borders
    # We add 1's as obstacles
    array[0:7,[-6, -5]] = 1
    array[-4:-1,-6] = 1
    array[-3:-1,-7] = 1
    array[-2:-1,-8] = 1
    array[6:8,4:9] = 1
    array[8:10,7:9] = 1
    array[1:3,6:8] = 1
    return array
                 


if __name__ == "__main__" :
    WF = WaveFrontAlgo()
    map = np.zeros((10,10))
    map[2,0] = 1
    print(map[4,5])
    goal = [2,17]
    start = [0,0]
    
    print(testing_array())
    result = WF.wavefront_planner_euclidean(testing_array(),goal) 
    print(np.around(result))
    local_min = WF.find_lcoal_minimum(result)
    print(local_min)

    # path = WF.find_the_path_8(result, start)
    # print(path)