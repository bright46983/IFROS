import numpy as np

class BushFireAlgo():
    def __init__(self) -> None:
        pass

    def isValid(self,index, map):
        try: #Check out of bound for exceeded index
            if index[0] < 0 or index[1] < 0: #Check out of bound for -1 index
                return False
            
            if map[index[0],index[1]] == 0: # Check if it a free space
                return True
            else: # apart from value 1 is valid
                return False
        except:
                return False
                        

    def bushfire_connect_4(self, grid_map:np.array):
        """
        Create Bushfire function with 4-connectivity 
        """
        motions = [[-1,0],[1,0],[0,-1],[0,1]] # Up, Down, Left, Right
        current_value = 1

        while np.any(grid_map==0):
            new_value = current_value +1
            for i in range(grid_map.shape[0]):
                for j in range(grid_map.shape[1]):
                    if grid_map[i,j] == current_value:
                        for m in motions:
                            index = [i+m[0],j+m[1]]
                            if self.isValid(index,grid_map):
                                grid_map[index[0],index[1]] = new_value
            
            current_value = new_value

        return grid_map
    
    def bushfire_connect_8(self, grid_map:np.array):
        """
        Create Bushfire function with 4-connectivity 
        """
        motions = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]] 
        current_value = 1

        while np.any(grid_map==0):
            new_value = current_value +1
            for i in range(grid_map.shape[0]):
                for j in range(grid_map.shape[1]):
                    if grid_map[i,j] == current_value:
                        for m in motions:
                            index = [i+m[0],j+m[1]]
                            if self.isValid(index,grid_map):
                                grid_map[index[0],index[1]] = new_value
            
            current_value = new_value

        return grid_map
                        
    def repulsive_function(self,grid_map:np.array, q:float):
        """
        Create repulsive function with Q threshold 
        """
        for i in range(grid_map.shape[0]):
                for j in range(grid_map.shape[1]):
                    if grid_map[i,j] == 1:
                        pass
                    elif grid_map[i,j] <= q:
                        grid_map[i,j] = 4*((1/grid_map[i,j]) - (1/q))**2
                    else:
                        grid_map[i,j] = 0

                    
        return grid_map
                 


if __name__ == "__main__" :
    BF = BushFireAlgo()
    map = np.zeros((10,10))
    map[2,0] = 1
  
    
    print(map)
    result = BF.bushfire_connect_4(map)
    print(result)
    result_rep = BF.repulsive_function(result,4)
    print(np.around(result_rep))
    

    # path = WF.find_the_path_8(result, start)
    # print(path)