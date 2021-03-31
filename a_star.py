def a_star_search(map):
    cell_map = []
    for i in range(len(map)):
        row = []
        for j in range(len(map[0])):
            row.append(Cell(i,j))
        cell_map.append(row)

    def print_cell_map():
        for i in range(len(cell_map)):
            row = []
            string = ''
            for j in range(len(cell_map[0])):
                string += str(cell_map[i][j].x) + str(cell_map[i][j].y) + " "
                
            print(string)
            print("\n")
            cell_map.append(row)
    print_cell_map()
    return cell_map


class Cell():
    def __init__(self,x,y, parent = None):
        self.x = x
        self.y = y
        self.parent = parent
        self.f_cost = -1
        self.g_cost = -1
        self.h_cost = -1 
    
def isUnblocked(x,y):
    return self.occdata[x][y] <= 0

def isDestination(x,y,dest):
    return x == dest.x and y == dest.y

def calculateHValue(x,y,dest):
    return sqrt((x - dest.x)**2 + (y-dest.y)**2)

def tracePath(dest):
    path = []
    cell = dest

    while cell != None:
        path.insert(0,(cell.x,cell.y))
        cell = cell.parent
    print(path)


a = Cell(0,0,None)
b = Cell(1,1,a)
c = Cell(2,2,b)

print(tracePath(c))

map = [ [(0,0), (1,0), (2,0)],
        [(1,0), (1,1), (2,1)],
        [(2,0), (2,1), (2,2)],
]

a_star_search(map)