def read_input():
	input_handle = open("input.txt", "r")
	contents = input_handle.readlines()
	algorithm = contents[0].rstrip()
	start_y = int(contents[2].split(' ')[0])
	start_x = int(contents[2].split(' ')[1])
	threshold = int(contents[3].rstrip())
	number_of_targets = int(contents[4].rstrip())
	target_locations = []
	for target in range(1, number_of_targets + 1):
		target_location = contents[4 + target].rstrip()
		target_y = target_location.split(' ')[0]
		target_x = target_location.split(' ')[1]
		target_locations.append(target_y+','+target_x)
	columns = int(contents[1].split(' ')[0])
	rows = int(contents[1].rstrip().split(' ')[1])
	matrix = []
	for row in range(rows):
		a = []
		row_values = contents[5 + number_of_targets + row].rstrip().split(' ')
		for col in range(columns):
			a.append(int(row_values[col]))
		matrix.append(a)
	config = {
	'algorithm': algorithm,
	'rows': rows,
	'columns': columns,
	'start_x': start_x,
	'start_y': start_y,
	'landing_site': str(start_y)+','+str(start_x),
	'elevation_threshold': threshold,
	'number_of_targets': number_of_targets,
	'target_locations': target_locations,
	'target_layout': matrix
	}
	return config

def get_output_path(target, parent_child_map, start_loc):
	path = [target]
	current_child = target
	while current_child != start_loc:
		path.append(parent_child_map[current_child])
		current_child = parent_child_map[current_child]
	return path

def get_adjacent_cells_bfs(max_rows, max_cols, x, y, matrix, threshold):
	adjacent_cells = []
	adjacent_vectors = [[-1,0],[1,0],[0,1],[0,-1],[-1,-1],[-1,1],[1,-1],[1,1]]
	for adj_vec in adjacent_vectors:
		changed_x = x+adj_vec[0]
		changed_y = y+adj_vec[1]
		if changed_x < 0 or changed_x >= max_rows or changed_y < 0 or changed_y >= max_cols:
			continue
		if abs(matrix[x][y] - matrix[changed_x][changed_y]) <= threshold:
			adjacent_cells.append(str(changed_y)+','+str(changed_x))
	return adjacent_cells

def get_adjacent_cells_ucs(max_rows, max_cols, x, y, matrix, threshold, path_cost):
	adjacent_cells = []
	adjacent_vectors = [[-1,0,10],[1,0,10],[0,1,10],[0,-1,10],[-1,-1,14],[-1,1,14],[1,-1,14],[1,1,14]]
	for adj_vec in adjacent_vectors:
		changed_x = x+adj_vec[0]
		changed_y = y+adj_vec[1]
		if changed_x < 0 or changed_x >= max_rows or changed_y < 0 or changed_y >= max_cols:
			continue
		if abs(matrix[x][y] - matrix[changed_x][changed_y]) <= threshold:
			adjacent_cells.append([str(changed_y)+','+str(changed_x), adj_vec[2]+path_cost])
	return adjacent_cells

def admissible_heuristic(current_x, current_y, target_x, target_y):
	x_diff = abs(current_x - target_x)
	y_diff = abs(current_y - target_y)
	return ((min(x_diff, y_diff)*14) + (abs(x_diff - y_diff)*10))

def get_adjacent_cells_astar(max_rows, max_cols, x, y, matrix, threshold, path_cost, target_y, target_x):
	adjacent_cells = []
	adjacent_vectors = [[-1,-1,14],[-1,1,14],[1,-1,14],[1,1,14],[-1,0,10],[1,0,10],[0,1,10],[0,-1,10]]
	for adj_vec in adjacent_vectors:
		changed_x = x+adj_vec[0]
		changed_y = y+adj_vec[1]
		if changed_x < 0 or changed_x >= max_rows or changed_y < 0 or changed_y >= max_cols:
			continue
		if abs(matrix[x][y] - matrix[changed_x][changed_y]) <= threshold:
			adjacent_cells.append([str(changed_y)+','+str(changed_x), adj_vec[2]+path_cost+abs(matrix[x][y] - matrix[changed_x][changed_y])+admissible_heuristic(changed_x, changed_y, target_x, target_y)])
	return adjacent_cells



def write_output_path(pathlist):
	file_out = open("output.txt", "w")
	newline = ''
	for path in pathlist:
		if 'FAIL' in path:
			file_out.write(newline+path)
			newline = '\n'
		else:
			temp_path = path[::-1]
			file_out.write(newline+' '.join(temp_path))
			newline = '\n'

def bfs(config):
	path_to_targets = []
	for target in config['target_locations']:
		frontier = {config['landing_site']: 0}
		visited = {}
		parent_child_map = {}
		path = []
		if config['landing_site'] == target:
			path.append(config['landing_site'])
		else:
			while frontier:
				current_node = next(iter(frontier.items()))
				del frontier[current_node[0]]
				visited[current_node[0]] = current_node[1]
				adj = get_adjacent_cells_bfs(config['rows'], config['columns'], int(current_node[0].split(',')[1]), int(current_node[0].split(',')[0]), config['target_layout'], config['elevation_threshold'])
				for child in adj:
					if child == target:
						parent_child_map[child] = current_node[0]
						path = get_output_path(child, parent_child_map, config['landing_site'])
						break
					elif child in frontier.keys() or child in visited.keys():
						continue
					else:
						parent_child_map[child] = current_node[0]
						frontier[child] = 0
				if path:
					break
		if not path:
			path_to_targets.append('FAIL')
		else:
			path_to_targets.append(path)
	return path_to_targets




def ucs(config):
	path_to_targets = []
	for target in config['target_locations']:
		frontier = {config['landing_site']: 0}
		visited = {}
		parent_child_map = {}
		path = []
		if config['landing_site'] == target:
			path.append(config['landing_site'])
		else:
			while frontier:
				current_node = min(frontier, key = frontier.get)
				current_node_cost = frontier[current_node]
				del frontier[current_node]
				if current_node == target:
					path = get_output_path(current_node, parent_child_map, config['landing_site'])
					break
				adj = get_adjacent_cells_ucs(config['rows'], config['columns'], int(current_node.split(',')[1]), int(current_node.split(',')[0]), config['target_layout'], config['elevation_threshold'], current_node_cost)
				for child in adj:
					if child[0] not in frontier.keys() and child[0] not in visited.keys():
						frontier[child[0]] = child[1]
						parent_child_map[child[0]] = current_node
					elif child[0] in frontier.keys() and child[1] < frontier[child[0]]:
						frontier[child[0]] = child[1]
						parent_child_map[child[0]] = current_node
					elif child[0] in visited.keys() and child[1] < visited[child[0]]:
						del visited[child[0]]
						frontier[child[0]] = child[1]
						parent_child_map[child[0]] = current_node
				visited[current_node] = current_node_cost
		if not path:
			path_to_targets.append('FAIL')
		else:
			path_to_targets.append(path)
	return path_to_targets



def astar(config):
	path_to_targets = []
	for target in config['target_locations']:
		frontier = {config['landing_site']: 0}
		visited = {}
		parent_child_map = {}
		path = []
		if config['landing_site'] == target:
			path.append(config['landing_site'])
		else:
			while frontier:
				current_node = min(frontier, key = frontier.get)
				current_node_cost = frontier[current_node]
				del frontier[current_node]
				if current_node == target:
					path = get_output_path(current_node, parent_child_map, config['landing_site'])
					break
				adj = get_adjacent_cells_astar(config['rows'], config['columns'], int(current_node.split(',')[1]), int(current_node.split(',')[0]), config['target_layout'], config['elevation_threshold'], current_node_cost, int(target.split(',')[0]), int(target.split(',')[1]))
				for child in adj:
					if child[0] not in frontier.keys() and child[0] not in visited.keys():
						frontier[child[0]] = child[1]
						parent_child_map[child[0]] = current_node
					elif child[0] in frontier.keys() and child[1] < frontier[child[0]]:
						frontier[child[0]] = child[1]
						parent_child_map[child[0]] = current_node
					elif child[0] in visited.keys() and child[1] < visited[child[0]]:
						del visited[child[0]]
						frontier[child[0]] = child[1]
						parent_child_map[child[0]] = current_node[0]
				visited[current_node] = current_node_cost
		if not path:
			path_to_targets.append('FAIL')
		else:
			path_to_targets.append(path)
	return path_to_targets

if __name__=="__main__":
	config = read_input()
	algorithm_dispatcher = {'BFS': bfs, 'UCS': ucs, 'A*': astar}
	path = algorithm_dispatcher[config['algorithm']](config)
	write_output_path(path)

	

