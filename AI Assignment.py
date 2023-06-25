class Node():
    def __init__(self, coordinate = None, rubbish_weight_volume = (0,0), is_disposal_room = False, parent = None, f = 0, g = 0, h = 0):
        self.coordinate = coordinate
        self.rubbish_weight_volume = rubbish_weight_volume
        self.is_disposal_room = is_disposal_room
        self.parent = parent
        self.f = g + h
        self.g = g
        self.h = h
        self.children = []
    
    def __str__(self):
        #return "%s, %s, %d, %s, %d, %d, %d" % (self.coordinate, self.rubbish_weight_volume, self.is_disposal_room, self.parent, self.f, self.g, self.h)
        return f"{self.coordinate}, {self.rubbish_weight_volume}, {self.is_disposal_room}, [{self.parent}], {self.f}, {self.g}, {self.h}"

    def add_children(self, children):
        self.children.extend(children)

# Function to calculate distance
def calculate_distance(state1, state2):
    x1, y1, z1 = state1[0]
    x2, y2, z2 = state2[0]
    
    distance = max(abs(x2 - x1), abs(y2 - y1), abs(z2 - z1))
    return distance

def move_up(coordinate):
    x, y, z = coordinate
    return (x, y+1, z-1) 

def move_up_right(coordinate):
    x, y, z = coordinate
    return (x+1, y, z-1)

def move_down_right(coordinate):
    x, y, z = coordinate
    return (x+1, y-1, z)
    
def move_down(coordinate):
    x, y, z = coordinate
    return (x, y-1, z+1)

def move_down_left(coordinate):
    x, y, z = coordinate
    return (x-1, y, z+1)

def move_up_left(coordinate):
    x, y, z = coordinate
    return (x-1, y+1, z)
    
def expand_and_return_children(state_space, node, goal_state):
    children = []
    
    available_actions = [
        move_up(node.coordinate), 
        move_up_right(node.coordinate), 
        move_down_right(node.coordinate),
        move_down(node.coordinate),
        move_down_left(node.coordinate),
        move_up_left(node.coordinate)
    ]
    
    '''       
    coords = [state[0] for state in state_space]
    for actions in available_actions:
        if actions in coords:
            # Create node
            i = coords.index(actions)
            print(state_space[i])
    ''' 
    
    for i, state in enumerate(state_space):
        if state[0] in available_actions:
            #print(state)
            children.append(Node(state[0], state[1], state[2], node, 0, calculate_distance(initial_state, state), calculate_distance(state, goal_state)))

    return children
    # return [str(child) for child in children]     

def append_and_sort(frontier, node):
    # Flag to indicate if the leaf node replaces a previous node
    replaced = False
    
    # Check if the current node has been expanded previously
    for (i,f) in enumerate(frontier):
        if f.coordinate == node.coordinate:
            if f.f > node.f:
                # Replace the existing node with the current node which has lower cost
                frontier[i] = node
                replaced = True
            else:
                return frontier
    
    # If the frontier has not been replaced with the leaf node, append the leaf node
    if not replaced:
        frontier.append(node)
    
    # Sort the frontier based on their f value
    sorted_frontier = sorted(frontier, key=lambda x: (x.f, -x.g))
    return sorted_frontier

# Create node in a-star function
def a_star(state_space, initial_state, goal_state):
    frontier = []
    explored = []
    found_goal = False
    goal = Node()
    path = []
    cost = 0
    rubbish_weight_volume = [0,0]

    frontier.append(Node(initial_state[0], initial_state[1], initial_state[2], None, 0, calculate_distance(initial_state, initial_state), calculate_distance(initial_state, goal_state)))

    while not found_goal:
        # Goal Test
        if frontier[0].coordinate == goal_state[0]:
            found_goal = True
            goal = frontier[0]
            # Check if its disposal room, if not, set the rubbish weight and volume to the current room's rubbish weight and volume
            if not (goal_state[2] == True):
                rubbish_weight_volume = frontier[0].rubbish_weight_volume
            break
    
        children = expand_and_return_children(state_space, frontier[0], goal_state)
        
        frontier[0].add_children(children)
        explored.append(frontier[0])
        
        del frontier[0]
        
        for child in children:
            if not (child.coordinate in [state.coordinate for state in explored]):
                frontier = append_and_sort(frontier, child)
        
        print("Explored: ", [str(explored) for explored in explored], "\n")
        print("Children: ", [str(child) for child in children], "\n")
        print("Frontier: ", [str(frontier) for frontier in frontier], "\n")
        print("---------------------------------------------\n")

    # Backtrack to find the path
    path = [goal.coordinate]
    cost = goal.f
    while goal.parent is not None: 
        path.insert(0, goal.parent.coordinate)
        for e in explored:
            if e.coordinate == goal.parent.coordinate:
                goal = e
                break
            
    return path, cost, rubbish_weight_volume
    
if __name__ == '__main__':    

    state_space = [
        # Coordinate, Rubbish Weight + Rubbish Volume, IsDisposalRoom
        [(0, 0, 0), (0, 0), False],
        [(0, -1, 1), (0, 0), False],
        [(1, -1, 0), (0, 0), False],
        [(1, 0, -1), (0, 0), False],
        [(2,-1,-1), (0, 0), False],
        [(2,-2,0), (0, 0), False],
        [(3,-2,-1), (5, 1), False],
        [(3, -1, -2), (0, 0), False],
        [(1, -2, 1), (0, 0), False],
    ]
    
    initial_state = state_space[0]
    
    path, cost, rubbish_weight_volume = a_star(state_space, initial_state, state_space[6])
    print(path)
    print(cost)
    print(rubbish_weight_volume)
    

    '''''''''
    [(1,-2,1), 0, 0, False],
    [(0,-2,2), 0, 0, False],
    [(4,-2,-2), 0, 0, False],
    [(4,-3,-1), 0, 0, False],
    [(5,-2,-3), 0, 0, True],

    (rubbish bin logic in dynamic search)
    -have rubbish bin var in the main algo
    -if a* return rubbish weight vol value not zero
    -add value to rubbish bin var
    -if a* return rubbish with weight vol zero, change var to zero
    -this implies the rubbish has been disposed
    '''''''''