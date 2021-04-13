def a_star_search(map,start,dest):
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
    open_list = []

    #Initializing parameters of starting node
    print("start is here")
    i = start[0]
    j = start[1]
    print((i,j))
    print(dest)
    print(len(cell_map) , len(cell_map[0]))
    print(cell_map[i][j])
    cell_map[i][j].parent_i = i
    cell_map[i][j].parent_j = j
    cell_map[i][j].f = 0
    cell_map[i][j].g = 0
    cell_map[i][j].h = 0
    print("start is here 1")

    #Insert starting position to open list and set f value to 0
    open_list.append((0,(i,j)))

    while open_list:
        current = popOpenList(open_list)
        for neighbor in get_neighbors(current):
            print(current, neighbor, "Hey i passedd this")
            x = neighbor[0]
            y = neighbor[1]
            if(isValid(x,y, len(cell_map), len(cell_map[0])) and isUnblocked(cell_map, x, y, len(cell_map), len(cell_map[0]))):
                if isFrontier(cell_map, x, y):
                    print("I terminated early coz found frontier", current , x, y)
                    cell_map[x][y].parent_i = current[1][0]
                    cell_map[x][y].parent_j = current[1][1]
                    return tracePath(cell_map, (x,y))
                elif (reached(neighbor,dest)):
                    #print("At", x, y, "Hey parent is updated,", cell_map[x][y].parent_i, cell_map[x][y].parent_j, " to ", current[1][0], current[1][1])
                    print("Destination cell is found")
                    cell_map[x][y].parent_i = current[1][0]
                    cell_map[x][y].parent_j = current[1][1]
                    #print(get_parent(cell_map,x,y))
                    #print_parents()
                    return tracePath(cell_map, dest)
                elif (x, y) not in open_list and cell_map[x][y].inClosedList == False:
                    gNew = cell_map[x][y].g + calculateHValue(x, y, start)
                    hNew = calculateHValue(x, y, dest)
                    fNew = gNew + hNew
                    if fNew < cell_map[x][y].f or cell_map[x][y].f == -1: 
                        cell_map[x][y].f = fNew
                        cell_map[x][y].g = gNew
                        cell_map[x][y].h = hNew
                        #print("At", x, y, "Hey parent is updated,", cell_map[x][y].parent_i, cell_map[x][y].parent_j, " to ", current[1][0], current[1][1])
                        cell_map[x][y].parent_i = current[1][0]
                        cell_map[x][y].parent_j = current[1][1]
                        #print(get_parent(cell_map,x,y))
                        #print("Hey f cost is updated " , fNew)
                        insertOpenList(open_list, (fNew , (x,y)))
                        #print("Open list is, ", open_list)
        cell_map[current[1][0]][current[1][1]].inClosedList = True
    print("Destination not found")
    return []
    
def get_parent(cell_map ,x ,y):
    #print("getting parent")
    return (cell_map[x][y].parent_i , cell_map[x][y].parent_j)

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

def neighbors(i,j):
    return ((i,j),(i,j+1), (i + 1,j + 1), (i + 1,j), (i+1,j -1 ), (i,j -1) , (i-1,j-1), (i-1,j), (i-1,j + 1))

def reached(current,dest):
    return current[0] == dest[0] and current[1] == dest[1]

def insertOpenList(anyList, element):
    if not(anyList):
        anyList.append(element)
        return
    i = 0
    while i < len(anyList) and element > anyList[i]:
        i+=1
    anyList.insert(i,element)

def popOpenList(anyList):
    return anyList.pop(0)

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
    minimum = min(row,col)
    return x >= 0 and x < minimum and y >= 0 and y < minimum

def isValidOcc(x,y,row,col):
    return x >= 0 and x < col and y >= 0 and y < row

def isDestination(x,y,dest):
    return x == dest.x and y == dest.y

def calculateHValue(x,y,dest):
    return ((x - dest[0])**2 + (y-dest[1])**2)**0.5

def tracePath(cell_map, dest):
    path = []
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


