class Node():
    '''This class creates a node to represent the rooms, storing the relevant information tailored to 
    each room as well as the F, G and H values for the A* algorithm.'''
    def __init__(self, coordinate = None, rubbish_weight_volume = (0,0), is_disposal_room = False, parent = None, f = 0, g = 0, h = 0):
        self.coordinate = coordinate
        self.rubbish_weight_volume = rubbish_weight_volume
        self.is_disposal_room = is_disposal_room
        self.parent = parent
        self.f = g + h
        self.g = g
        self.h = h
        self.children = []
    
    # Modify the __str__ function to print the node in a readable format
    def __str__(self):
        #return "%s, %s, %d, %s, %d, %d, %d" % (self.coordinate, self.rubbish_weight_volume, self.is_disposal_room, self.parent, self.f, self.g, self.h)
        return f"{self.coordinate}, {self.rubbish_weight_volume}, {self.is_disposal_room}, [{self.parent}], {self.f}, {self.g}, {self.h}"

    # Function to add children to the node
    def add_children(self, children):
        self.children.extend(children)

# Function to calculate distance between 2 rooms
def calculate_distance(state1, state2):
    x1, y1, z1 = state1[0]
    x2, y2, z2 = state2[0]
    
    # Calculate distance by taking the maximum of the absolute difference between the x, y and z coordinates
    distance = max(abs(x2 - x1), abs(y2 - y1), abs(z2 - z1))
    return distance

# Function to move to the upper room
def move_up(coordinate):
    x, y, z = coordinate
    return (x, y+1, z-1) 

# Function to move to the upper right room
def move_up_right(coordinate):
    x, y, z = coordinate
    return (x+1, y, z-1)

# Function to move to the lower right room
def move_down_right(coordinate):
    x, y, z = coordinate
    return (x+1, y-1, z)
    
# Function to move to the lower room
def move_down(coordinate):
    x, y, z = coordinate
    return (x, y-1, z+1)

# Function to move to the lower left room
def move_down_left(coordinate):
    x, y, z = coordinate
    return (x-1, y, z+1)

# Function to move to the upper left room
def move_up_left(coordinate):
    x, y, z = coordinate
    return (x-1, y+1, z)
    
# Provide children of a node (the reachable adjacent rooms) given the state space
def expand_and_return_children(state_space, initial_state, node, goal_state):
    children = []
    
    # List of adjacent rooms from the current room
    available_actions = [
        move_up(node.coordinate), 
        move_up_right(node.coordinate), 
        move_down_right(node.coordinate),
        move_down(node.coordinate),
        move_down_left(node.coordinate),
        move_up_left(node.coordinate)
    ]
    
    # Check if the adjacent rooms are in the state space
    for state in state_space:
        if state[0] in available_actions:
            #print(state)
            # Create node for the valid adjacent rooms
            children.append(Node(state[0], state[1], state[2], node, 0, calculate_distance(initial_state, state), calculate_distance(state, goal_state)))

    return children

# Function to append a new frontier and sort the frontier
def append_and_sort(frontier, node):
    # Flag to indicate if the leaf node replaces a previous node
    replaced = False
    
    # Check if the current node has been expanded previously
    for (i,f) in enumerate(frontier):
        # If the current node has been expanded previously, check if the current node has lower cost
        if f.coordinate == node.coordinate:
            if f.f > node.f:
                # Replace the existing node with the current node which has lower cost
                frontier[i] = node
                replaced = True
            else:
                # If the current node has higher cost, do not append the current node
                return frontier
    
    # If the frontier has not been replaced with the leaf node, append the leaf node
    if not replaced:
        frontier.append(node)
    
    # Sort the frontier based on their f value in ascending order
    # If the f value is the same, sort the frontier based on their g value in descending order
    sorted_frontier = sorted(frontier, key=lambda x: (x.f, -x.g))
    return sorted_frontier

# Function to calculate whether rubbish bin will exceed capacity of 40kg or 5m^3
def is_over_capacity(bin_status, rubbish_room):
    # Calculate the weight and volume of the rubbish bin after moving to the next rubbish room
    weight = bin_status[0] + rubbish_room[1][0]
    volume = bin_status[1] + rubbish_room[1][1]
    
    # Check if the weight or volume of the rubbish bin exceeds the capacity
    if (weight > 40) or (volume > 5):
        print(f"Moving to the next rubbish room will result in the bin status = [{weight}, {volume}], exceeding the capacity of the rubbish bin.")
        return True
    else:
        return False

# A* Search Algorithm
def a_star(state_space, initial_state, goal_state):
    # Initialise frontier, explored, goal node, path, cost and rubbish weight and volume
    frontier = []
    explored = []
    found_goal = False
    goal = Node()
    path = []
    cost = 0
    rubbish_weight_volume = [0,0]

    # Create intial node and append to frontier
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
    
        # Expand the first node in the frontier list
        children = expand_and_return_children(state_space, initial_state, frontier[0], goal_state)
        
        # Add children list to the first node in the frontier list
        frontier[0].add_children(children)
        
        # Add the expanded frontier explored list
        explored.append(frontier[0])
        
        # Remove the expanded frontier
        del frontier[0]
        
        # Add children to the frontier
        for child in children:
            # Check if expanded node has been explored or generated previously
            if not (child.coordinate in [state.coordinate for state in explored]):
                frontier = append_and_sort(frontier, child)
        
        # Print the current state of the search
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
    
# Function to find the path of rubbish collection and disposal 
# Help to determine and switch the goal state for the A* Search Algorithm
def dynamic_search(state_space, initial_state):
    current_state = initial_state
    # Find the list of rubbish rooms and disposal rooms
    rubbish_list = [state for state in state_space if ((state[1][0] > 0) and (state[1][1] > 0))]
    disposal_list = [state for state in state_space if state[2] == True]
    visited_rubbish_list = []
    is_bin_empty = True
    goal_state = None
    bin_status = [0,0]
    full_path = []
    total_cost = []
    counter = 1

    # While there are still rubbish rooms or the rubbish bin is not empty
    while rubbish_list or not(is_bin_empty):
        print("=============================================\n")
        # Remove the last calculated distance between the current room and the rubbish room as well as the disposal room
        if counter > 1:
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
        
        if rubbish_list:
            # Check Bin Status, if moving to the next rubbish room and collecting rubbish will exceed the capacity,
            # go to the disposal room
            if bin_status[0] <= 40 and bin_status[1] <= 5 and not(is_over_capacity(bin_status, rubbish_list[0])):
                # Set the goal state to the nearest rubbish room
                goal_state = rubbish_list[0]
                print("Ronny will be collecting rubbish.")
                print(f"Path: {counter} | Current State: {current_state[0]} | Goal State: {goal_state[0]} (Rubbish Room) | Bin Status: {bin_status}\n")
                # Find the path using A* Search Algorithm
                path, cost, rubbish_weight_volume = a_star(state_space, current_state, goal_state)
                print(f"Ronny has collected rubbish with weight {rubbish_weight_volume[0]} kg and volume {rubbish_weight_volume[1]} m³.")
                # Append the path, cost to the full path and total cost list
                full_path.append(path)
                total_cost.append(cost)
                # Add the rubbish weight and volume to the rubbish bin
                bin_status[0] += rubbish_weight_volume[0]   
                bin_status[1] += rubbish_weight_volume[1]
                is_bin_empty = False
                visited_rubbish_list.append(goal_state)
                # Remove the visited rubbish room from the list
                del rubbish_list[0]
                # Set the current state to the goal state
                current_state = goal_state
            
            else:
                # Disposing rubbish if the bin is full
                # Set the goal state to the nearest disposal room
                goal_state = disposal_list[0]
                print("Hence, Ronny will be disposing rubbish.")
                print(f"Path: {counter} | Current State: {current_state[0]} | Goal State: {goal_state[0]} (Disposal Room) | Bin Status: {bin_status}\n")
                # Find the path using A* Search Algorithm
                path, cost, rubbish_weight_volume = a_star(state_space, current_state, goal_state)
                print("Ronny has disposed the rubbish.")
                # Append the path, cost to the full path and total cost list and set rubbish weight and volume to 0
                full_path.append(path)
                total_cost.append(cost)
                current_state = goal_state
                bin_status = rubbish_weight_volume
                is_bin_empty = True
                print(f"Disposal Room Ronny Visited: {goal_state[0]}")

        # To dispose any rubbish left when the all rubbish has been collected
        elif bin_status[0] > 0 or bin_status[1] > 0:
            goal_state = disposal_list[0]
            print("Since all rubbish is collected, Ronny will be disposing rubbish.")
            print(f"Path: {counter} | Current State: {current_state[0]} | Goal State: {goal_state[0]} (Disposal Room) | Bin Status: {bin_status}\n")
            path, cost, rubbish_weight_volume = a_star(state_space, current_state, goal_state)
            print("Ronny has disposed the rubbish.")            
            full_path.append(path)
            total_cost.append(cost)
            current_state = goal_state
            bin_status = rubbish_weight_volume
            is_bin_empty = True
            print(f"Disposal Room Ronny Visited: {goal_state[0]}")

        else:
            break
        
        
        print(f"Visited Rubbish Room: {[visited_room[0] for visited_room in visited_rubbish_list]}")
        print(f"Rubbish Room Left: {[unvisited_room[0] for unvisited_room in rubbish_list]}")
        print(f"Updated Bin Status: {bin_status}\n")
        print("=============================================")
        # Increment the counter to indicate the number of iterations
        counter += 1

    print("Ronny has successfully collected and disposed all rubbish.\n")
    return full_path, total_cost

# Function to display the full path and cost
def display(full_path, full_cost):
    print("Displaying the full path and cost...\n ")
    for path, cost in zip(full_path, full_cost):
        print(f"Path {full_path.index(path) + 1}: {path}")
        print(f"Cost: {cost}\n")
    
    # Display the full path and cost
    print(f"Total Cost: {sum(full_cost)}")

    
if __name__ == '__main__':    

    # Define the state space, this particular sequence of the states (go by columns) returns a total cost of 41
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
    
    # # Uncomment this block of code to test this state space
    # # State space that returns a total cost of 40 (go by row)
    # state_space = [
    #     # Coordinate, Rubbish Weight + Rubbish Volume, is_disposal_room
    #     [(0, 0, 0), (0, 0), False],
    #     [(1, 0, -1), (0, 0), False],
    #     [(2, -1, -1), (0, 0), False],
    #     [(3, -1, -2), (0, 0), False],
    #     [(4, -2, -2), (0, 0), False],
    #     [(5, -2, -3), (0, 0), True],
    #     [(5, -5, 0), (0, 0), False],
    #     [(6, -3, -3), (0, 0), False],
    #     [(7, -3, -4), (30, 1), False],
    #     [(8, -4, -4), (0, 0), False],
    #     [(0, -1, 1), (0, 0), False],
    #     [(1, -1, 0), (0, 0), False],
    #     [(2, -2, 0), (0, 0), False],
    #     [(3, -2,-1), (5, 1), False],
    #     [(4, -3, -1), (0, 0), False],
    #     [(6, -4, -2), (10, 2), False],
    #     [(7, -4, -3), (0, 0), False],
    #     [(8, -5, -3), (10, 3), False],
    #     [(5, -3, -2), (0, 0), False],
    #     [(0, -2, 2), (0, 0), False],
    #     [(1, -2, 1), (0, 0), False],
    #     [(2, -3, 1), (5, 1), False],
    #     [(3, -3, 0), (0, 0), False],
    #     [(4, -4, 0), (10, 2), False],
    #     [(5, -4, -1), (0, 0), False],
    #     [(6, -5, -1), (0, 0), False],
    #     [(7, -5, -2), (0, 0), False],
    #     [(8, -6, -2), (0, 0), False], 
    #     [(0, -3, 3), (0, 0), False],
    #     [(1, -3, 2), (30, 3), False],
    #     [(2, -4, 2), (0, 0), False],
    #     [(3, -4, 1), (0, 0), False],
    #     [(4, -5, 1), (0, 0), False], 
    #     [(5, -6, 1), (0, 0), False],
    #     [(6, -6, 0), (0, 0), False],
    #     [(7, -6, -1), (20, 2), False],
    #     [(8, -7, -1), (0, 0), False],
    #     [(0, -4, 4), (0, 0), False],
    #     [(1, -4, 3), (0, 0), False],
    #     [(2, -5, 3), (0, 0), False],
    #     [(3, -5, 2), (5, 3), False],
    #     [(4, -6, 2), (20, 1), False],
    #     [(5, -7, 2), (0, 0), False],
    #     [(6, -7, 1), (5, 2), False],
    #     [(7, -7, 0), (0, 0), False],
    #     [(8, -8, 0), (0, 0), False],
    #     [(0, -5, 5), (10, 1), False],
    #     [(1, -5, 4), (0, 0), False],
    #     [(2, -6, 4), (0, 0), True],
    #     [(3, -6, 3), (0, 0), False],
    #     [(4, -7, 3), (0, 0), False],
    #     [(6, -8, 2), (0, 0), False],
    #     [(7, -8, 1), (0, 0), False],
    #     [(8, -9, 1), (0, 0), True]
    # ]
    
    initial_state = state_space[0]
    
    print("Welcome to Ronny's Rubbish Room's Maze Clearing!")
    print("The program will assist Ronny in finding the path to collect and dispose all the rubbish in the maze.")
    print("The rubbish bin will be represented by the weight (kg) followed by the volume (m³) of the rubbish.")
    print("Let's start!")
    path, cost = dynamic_search(state_space, initial_state)
    display(path, cost)

