import heapq
import numpy as np

class PriorityQueue:
    def __init__(self):
        self.queue = []

    def isEmpty(self):
        return not(self.queue)

    def insert(self,item, priority):
        heapq.heappush(self.queue, (priority , item))

    def pop(self):
        return heapq.heappop(self.queue)

    def exist(self, item):
        return item in self.queue
    
    def print(self):
        for i in self.queue:
            print(i)


def a_star_search(map,start,dest):

    debugging = False

    map = np.asarray(map)
    map = map.transpose()

    map_height = len(map)
    map_width = len(map[0])

    def print_cell_map():
        cell = []
        for i in range(map_height):
            row = []
            for j in range(map_width):
                row.append(cell_map[i][j].occ_value)  
            cell.append(row)
            #print(row)
            #print("\n")
        return cell

    def print_parents():
        for i in range(len(map_height)):
            row = []
            for j in range(map_width):
                row.append((cell_map[i][j].parent_i , cell_map[i][j].parent_j))  
            print(row)
            print("\n")

    cell_map = []

    # Converting map of integers to map of Cell objects
    for i in range(map_height):
        row = []
        for j in range(map_width):
            #print( i , j , map_width , map_height)
            row.append(Cell(i,j,map[i][j]))
        cell_map.append(row)

    # cell = print_cell_map()
    # cell[86][63] = 100
    # np.savetxt('cell.txt', cell, fmt='%d', delimiter='')
    # return
    
    # Initializing open list 
    open_list = PriorityQueue()

    #Initializing parameters of starting node
    i = start[0]
    j = start[1]
    #print(map_width , map_height)
    cell_map[i][j].parent_i = i
    cell_map[i][j].parent_j = j
    cell_map[i][j].f = 0
    cell_map[i][j].g = 0
    cell_map[i][j].h = 0
    print( start , dest, "initializing")

    #Insert starting position to open list and set f value to 0
    open_list.insert(item = start, priority = 0)
    open_list.print()

    while not(open_list.isEmpty()):
        current = open_list.pop()
        origin = current[1]
        #print(origin)
        for neighbor in get_neighbors(origin, map_height , map_width):
            x = neighbor[0]
            y = neighbor[1]
            #print(origin, "Hey i passedd this" , x ,y)
            print( x, y , isUnblocked(cell_map, x, y, map_height, map_width))
            if isValid(x,y, map_height, map_width) and isUnblocked(cell_map, x, y, map_height, map_width):
                #print(" is valid " , x , y , neighbor)
                # if isFrontier(cell_map, x, y):
                #     print("I terminated early coz found frontier", current , y, x)
                #     if debugging:
                #         print("At", y, x, "Hey parent is updated,", cell_map[x][y].parent_j, cell_map[x][y].parent_i, " to ", origin[0], origin[1])
                #     cell_map[x][y].parent_i = origin[0]
                #     cell_map[x][y].parent_j = origin[1]
                #     if debugging:    
                #         print(get_parent(cell_map,y,x))
                #     return tracePath(cell_map, (y,x))
                if (reached(neighbor,dest)):
                    if debugging:
                        print("At", x, y, "Hey parent is updated,", cell_map[x][y].parent_i, cell_map[x][y].parent_j, " to ", origin[0], origin[1])
                    print("Destination cell is found")
                    cell_map[x][y].parent_i = origin[0]
                    cell_map[x][y].parent_j = origin[1]
                    if debugging:
                        print(get_parent(cell_map,x,y))
                    return tracePath(cell_map, dest, map_height , map_width)
                elif not(open_list.exist((x, y))) and cell_map[x][y].inClosedList == False:
                    gNew = cell_map[x][y].g + calculateEuclidean( origin , neighbor)
                    hNew = calculateHValue((x, y), dest)
                    fNew = gNew + hNew
                    if fNew < cell_map[x][y].f or cell_map[x][y].f == -1: 
                        cell_map[x][y].f = fNew
                        cell_map[x][y].g = gNew
                        cell_map[x][y].h = hNew
                        if debugging:
                            print("At", x, y, "Hey parent is updated,", cell_map[x][y].parent_i, cell_map[x][y].parent_j, " to ", origin[0], origin[1])
                        cell_map[x][y].parent_i = origin[0]
                        cell_map[x][y].parent_j = origin[1]
                        if debugging:
                            print(get_parent(cell_map,x,y))
                        #print("Hey f cost is updated " , fNew)
                        open_list.insert(priority = fNew , item = (x,y))
                        #print("Open list is, ", open_list)
        cell_map[origin[1]][origin[0]].inClosedList = True
    print("Destination not found")
    return []
    
def get_parent(cell_map ,x ,y):
    print("getting parent")
    return (cell_map[x][y].parent_j , cell_map[x][y].parent_i)

class Cell():
    def __init__(self,x,y,occ_value):
        self.x = x
        self.y = y
        self.parent_i = -1
        self.parent_j = -1
        self.f = -1        #Set -1  value as undeclared
        self.g = -1
        self.h = -1
        self.inClosedList = False
        self.occ_value = occ_value
    

def get_neighbors(current, row , col):
    x, y = current
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            position = (x + dx, y + dy)
            #print(position[0] , position[1])
            #print(isValid(position[0] , position[1] , row , col))
            #print( row , col)
            if isValid(position[0] , position[1] , row , col):
                #print(position, "hey position")
                yield position


def reached(current,dest):
    return current[0] == dest[0] and current[1] == dest[1]

def isUnblocked(cell_map,i ,j, row , col):
    neighbors = ((i -1 ,j + 1 ) , (i ,j + 1), ( i + 1,j + 1), (i - 1, j ), ( i ,j  ), ( i + 1 , j ) , (i - 1,j - 1), ( i ,j - 1), ( i + 1,j - 1))

    print(f' I now at {i} , {j} with {neighbors} ' )
    for (a,b) in neighbors:
        #print( a,b)
        #print(cell_map[a][b].occ_value , "im in unblocked")
        if isValid(a, b, row, col):
            if cell_map[a][b].occ_value == 3:
                return False
    return True

def isFrontier(cell_map, i,j):
    neighbors = ((i -1 ,j + 1 ) , (i ,j + 1), ( i + 1,j + 1), (i - 1, j ), ( i ,j  ), ( i + 1 , j ) , (i-1,j-1), ( i ,j -1), ( i + 1,j - 1))
    for (x,y) in neighbors:
        if cell_map[x][y].occ_value in (2,3):
            return False
    return True

def isValid(x,y, row , col):
    #print("in valid ")
    #print( x , y , row , col)
    return x >= 0 and x < row and y >= 0 and y < col

def calculateEuclidean(point1 , point2):
    return ( (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def calculateHValue( start,dest):
    return abs(dest[0] - start[0]) + abs(dest[1] - start[1])

def tracePath(cell_map, dest, row , col):
    path = []
    y = dest[1]
    x = dest[0]
    #print("trace path" , col , row)
    while not(cell_map[x][y].parent_i == x and cell_map[x][y].parent_j == y):
        #print(col,row)
        #print(cell_map[col][row].parent_i , cell_map[col][row].parent_j)
        path.insert(0,(x,y))
        #print("trace path" , col , row)
        temp_y = cell_map[x][y].parent_j
        temp_x = cell_map[x][y].parent_i
        y = temp_y
        x = temp_x
    #path.insert(0,(row,col))
    #print(path)
    # for (x,y) in path:
    #     print(f'{x} , {y} , is unblocked ? {isUnblocked(cell_map, x, y, row, col)} ')
    return path


map = [ [ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 ],
            [ 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 ],
            [ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 ],
            [ 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 ],
            [ 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 ],
            [ 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 ],
            [ 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 ],
            [ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 ],
            [ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 ] ]

start = [0,0]
dest = [8,8]


