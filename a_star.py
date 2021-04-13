import heapq

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

    def print_cell_map():
        for i in range(len(cell_map)):
            row = []
            for j in range(len(cell_map[0])):
                row.append(cell_map[i][j].f)  
            print(row)
            print("\n")

    def print_parents():
        for i in range(len(cell_map)):
            row = []
            for j in range(len(cell_map[0])):
                row.append((cell_map[i][j].parent_i , cell_map[i][j].parent_j))  
            print(row)
            print("\n")
    cell_map = []

    # Converting map of integers to map of Cell objects
    for i in range(len(map)):
        row = []
        for j in range(len(map[0])):
            row.append(Cell(i,j,map[i][j]))
        cell_map.append(row)

    # Initializing open list 
    open_list = PriorityQueue()

    #Initializing parameters of starting node
    i = start[1]
    j = start[0]
    a = dest[0]
    b = dest[1]
    dest = (a,b)
    cell_map[i][j].parent_i = i
    cell_map[i][j].parent_j = j
    cell_map[i][j].f = 0
    cell_map[i][j].g = 0
    cell_map[i][j].h = 0
    print((i,j ) , dest, "initializing")

    #Insert starting position to open list and set f value to 0
    open_list.insert(item = (i,j), priority = 0)
    open_list.print()

    while not(open_list.isEmpty()):
        current = open_list.pop()
        origin = current[1]
        print(origin)
        for neighbor in get_neighbors(origin, len(cell_map) , len(cell_map[0])):
            #print(origin, neighbor, "Hey i passedd this")
            x = neighbor[1]
            y = neighbor[0]
            if isValid(x,y, len(cell_map), len(cell_map[0])) and isUnblocked(cell_map, x, y, len(cell_map), len(cell_map[0])):
                if isFrontier(cell_map, x, y):
                    print("I terminated early coz found frontier", current , y, x)
                    if debugging:
                        print("At", y, x, "Hey parent is updated,", cell_map[y][x].parent_j, cell_map[y][x].parent_i, " to ", origin[0], origin[1])
                    cell_map[y][x].parent_i = origin[0]
                    cell_map[y][x].parent_j = origin[1]
                    if debugging:    
                        print(get_parent(cell_map,y,x))
                    return tracePath(cell_map, (y,x))
                elif (reached(neighbor,dest)):
                    if debugging:
                        print("At", y, x, "Hey parent is updated,", cell_map[y][x].parent_j, cell_map[y][x].parent_i, " to ", origin[0], origin[1])
                    print("Destination cell is found")
                    cell_map[y][x].parent_i = origin[0]
                    cell_map[y][x].parent_j = origin[1]
                    if debugging:
                        print(get_parent(cell_map,y,x))
                    return tracePath(cell_map, dest)
                elif not(open_list.exist((y, x))) and cell_map[y][x].inClosedList == False:
                    gNew = cell_map[y][x].g + calculateEuclidean( origin , neighbor)
                    hNew = calculateHValue((y, x), dest)
                    fNew = gNew + hNew
                    if fNew < cell_map[y][x].f or cell_map[y][x].f == -1: 
                        cell_map[y][x].f = fNew
                        cell_map[y][x].g = gNew
                        cell_map[y][x].h = hNew
                        if debugging:
                            print("At", y, x, "Hey parent is updated,", cell_map[y][x].parent_j, cell_map[y][x].parent_i, " to ", origin[0], origin[1])
                        cell_map[y][x].parent_i = origin[0]
                        cell_map[y][x].parent_j = origin[1]
                        if debugging:
                            print(get_parent(cell_map,y,x))
                        #print("Hey f cost is updated " , fNew)
                        open_list.insert(priority = fNew , item = (y,x))
                        #print("Open list is, ", open_list)
        cell_map[origin[0]][origin[1]].inClosedList = True
    print("Destination not found")
    return []
    
def get_parent(cell_map ,x ,y):
    print("getting parent")
    return (cell_map[y][x].parent_j , cell_map[y][x].parent_i)

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
    
def get_neighbors(current):
    i = current[1][0]
    j = current[1][1]
    return ((i,j) , (i,j+1), (i + 1,j + 1), (i + 1,j), (i+1,j -1 ), (i,j -1) , (i-1,j-1), (i-1,j), (i-1,j + 1))

def get_neighbors(current, row , col):
    x, y = current
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            position = (x + dx, y + dy)
            if isValid(position[0] , position[1] , row , col):
                yield position


def reached(current,dest):
    return current[0] == dest[0] and current[1] == dest[1]

def isUnblocked(cell_map,i ,j, row , col):
    neighbors = ((i,j) , (i,j+1), (i + 1,j + 1), (i + 1,j), (i+1,j -1 ), (i,j -1) , (i-1,j-1), (i-1,j), (i-1,j + 1))
    for (i,j) in neighbors:
        if cell_map[j][i].occ_value == 3:
            return False
    return True

def isFrontier(cell_map, i,j):
    neighbors = ((i,j) , (i,j+1), (i + 1,j + 1), (i + 1,j), (i+1,j -1 ), (i,j -1) , (i-1,j-1), (i-1,j), (i-1,j + 1))
    for (x,y) in neighbors:
        if cell_map[y][x].occ_value in (2,3):
            return False
    return True

def isValid(x,y, row , col):
    return x >= 0 and x < row and y >= 0 and y < col

def calculateEuclidean(point1 , point2):
    return ( (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def calculateHValue( start,dest):
    return abs(dest[0] - start[0]) + abs(dest[1] - start[1])

def tracePath(cell_map, dest):
    path = []
    print(dest)
    row = dest[0]
    col = dest[1]
    while not(cell_map[row][col].parent_i == row and cell_map[row][col].parent_j == col):
        path.insert(0,(row,col))
        temp_row = cell_map[row][col].parent_i
        temp_col = cell_map[row][col].parent_j
        row = temp_row
        col = temp_col
    #path.insert(0,(row,col))
    #print(path)
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


