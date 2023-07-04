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

# Function to calculate distance between 2 rooms
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
    
def expand_and_return_children(state_space, initial_state, node, goal_state):
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
    
    for state in state_space:
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

# Function to calculate whether rubbish bin will exceed capacity of 40kg or 5m^3
def is_over_capacity(bin_capacity, rubbish_room):
    if ((bin_capacity[0] + rubbish_room[1][0]) > 40) or ((bin_capacity[1] + rubbish_room[1][1]) > 5):
        return True
    else:
        return False

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
    
        children = expand_and_return_children(state_space, initial_state, frontier[0], goal_state)
        
        frontier[0].add_children(children)
        explored.append(frontier[0])
        
        del frontier[0]
        
        for child in children:
            if not (child.coordinate in [state.coordinate for state in explored]):
                frontier = append_and_sort(frontier, child)
        
        print("Explored: ", [explored.coordinate for explored in explored], "\n")
        print("Children: ", [child.coordinate for child in children], "\n")
        print("Frontier: ", [frontier.coordinate for frontier in frontier], "\n")
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
    
def dynamic_search(state_space, initial_state):
    current_state = initial_state
    rubbish_list = [state for state in state_space if ((state[1][0] > 0) and (state[1][1] > 0))]
    disposal_list = [state for state in state_space if state[2] == True]
    visited_rubbish_list = []
    is_bin_empty = True
    goal_state = None
    bin_capacity = [0,0]
    full_path = []
    total_cost = []
    counter = 0

    while rubbish_list or not(is_bin_empty):
        if counter > 0:
            for rubbish in rubbish_list:
                rubbish.pop()
                
            for disposal in disposal_list:
                disposal.pop()
        
        if rubbish_list:
            # Calculate the distance of current room to each of the rubbish room
            for rubbish_room in rubbish_list:
                rubbish_distance = calculate_distance(current_state, rubbish_room)
                rubbish_room.append(rubbish_distance)
            # Sort the rubbish list based on distance
            rubbish_list = sorted(rubbish_list, key = lambda x: x[3])
        
        # Calculate the distance of the current room to each disposal room
        for disposal_room in disposal_list:
            disposal_distance = calculate_distance(current_state, disposal_room)
            disposal_room.append(disposal_distance)
        # Sort the disposal list based on distance
        disposal_list = sorted(disposal_list, key = lambda x: x[3])

        # Check if there is still rubbish to be collected to avoid the situation where
        # all rubbish has been collected but not disposed yet 
        
        # Check bin capacity, if moving to the next rubbish room will exceed the capacity, go to the disposal room
        if rubbish_list:
            if bin_capacity[0] <= 40 and bin_capacity[1] <= 5 and not(is_over_capacity(bin_capacity, rubbish_list[0])):
                goal_state = rubbish_list[0]
                print(f"Goal state: {goal_state}\n")
                path, cost, rubbish_weight_volume = a_star(state_space, current_state, goal_state)
                full_path.append(path)
                total_cost.append(cost)
                bin_capacity[0] += rubbish_weight_volume[0]   
                bin_capacity[1] += rubbish_weight_volume[1]
                is_bin_empty = False
                visited_rubbish_list.append(goal_state)
                del rubbish_list[0]
                current_state = goal_state
            
            else:
                goal_state = disposal_list[0]
                print(f"Goal state: {goal_state}\n")
                path, cost, rubbish_weight_volume = a_star(state_space, current_state, goal_state)
                full_path.append(path)
                total_cost.append(cost)
                current_state = goal_state
                bin_capacity = rubbish_weight_volume
                is_bin_empty = True
                print(f"Visited disposal room: {goal_state}")

        elif bin_capacity[0] > 0 or bin_capacity[1] > 0:
            goal_state = disposal_list[0]
            print(f"Goal state: {goal_state}\n")
            path, cost, rubbish_weight_volume = a_star(state_space, current_state, goal_state)
            full_path.append(path)
            total_cost.append(cost)
            current_state = goal_state
            bin_capacity = rubbish_weight_volume
            is_bin_empty = True
        
        else:
            break
        
        print(f"Visited rubbish: {visited_rubbish_list}\n")
        print(f"Rubbish room left: {rubbish_list}\n")
        counter += 1
        print(f"Iteration: {counter}\n")
        print(f"Bin Capacity: {bin_capacity}\n")
        print(f"Current State: {current_state}\n")
        
    return full_path, sum(total_cost)
        
if __name__ == '__main__':    

    state_space = [
        # Coordinate, Rubbish Weight + Rubbish Volume, IsDisposalRoom
        [(0, 0, 0), (0, 0), False],
        [(0, -1, 1), (0, 0), False],
        [(0, -2, 2), (0, 0), False],
        [(0, -3, 3), (0, 0), False],
        [(0, -4, 4), (0, 0), False],
        [(0, -5, 5), (10, 1), False],
        [(1, 0, -1), (0, 0), False],
        [(1, -1, 0), (0, 0), False],
        [(1, -2, 1), (0, 0), False],
        [(1, -3, 2), (30, 3), False],
        [(1, -4, 3), (0, 0), False],
        [(1, -5, 4), (0, 0), False],
        [(2, -1, -1), (0, 0), False],
        [(2, -2, 0), (0, 0), False],
        [(2, -3, 1), (5, 1), False],
        [(2, -4, 2), (0, 0), False],
        [(2, -5, 3), (0, 0), False],
        [(2, -6, 4), (0, 0), True],
        [(3, -1, -2), (0, 0), False],
        [(3, -2,-1), (5, 1), False],
        [(3, -3, 0), (0, 0), False],
        [(3, -4, 1), (0, 0), False],
        [(3, -5, 2), (5, 3), False],
        [(3, -6, 3), (0, 0), False],
        [(4, -2, -2), (0, 0), False],
        [(4, -3, -1), (0, 0), False],
        [(4, -4, 0), (10, 2), False],
        [(4, -5, 1), (0, 0), False],
        [(4, -6, 2), (20, 1), False],
        [(4, -7, 3), (0, 0), False],
        [(5, -2, -3), (0, 0), True],
        [(5, -3, -2), (0, 0), False],
        [(5, -4, -1), (0, 0), False],
        [(5, -5, 0), (0, 0), False],
        [(5, -6, 1), (0, 0), False],
        [(5, -7, 2), (0, 0), False],
        [(6, -3, -3), (0, 0), False],
        [(6, -4, -2), (10, 2), False],
        [(6, -5, -1), (0, 0), False],
        [(6, -6, 0), (0, 0), False],
        [(6, -7, 1), (5, 2), False],
        [(6, -8, 2), (0, 0), False],
        [(7, -3, -4), (30, 1), False],
        [(7, -4, -3), (0, 0), False],
        [(7, -5, -2), (0, 0), False],
        [(7, -6, -1), (20, 2), False],
        [(7, -7, 0), (0, 0), False],
        [(7, -8, 1), (0, 0), False],
        [(8, -4, -4), (0, 0), False],
        [(8, -5, -3), (10, 3), False],
        [(8, -6, -2), (0, 0), False],
        [(8, -7, -1), (0, 0), False],
        [(8, -8, 0), (0, 0), False],
        [(8, -9, 1), (0, 0), True]
    ]
    
    initial_state = state_space[0]
    '''
    path, cost, rubbish_weight_volume = a_star(state_space, initial_state, state_space[6])
    print(path)
    print(cost)
    print(rubbish_weight_volume)'''
    
    path, cost = dynamic_search(state_space, initial_state)
    print(f"Path: {path}")
    print(f"Cost: {cost}")

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
