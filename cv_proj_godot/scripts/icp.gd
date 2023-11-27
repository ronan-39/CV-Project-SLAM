extends Node

# pc_1 is the source point cloud
# pc_2 is the reference point cloud
# n is the number of iterations
func icp(pc_1, pc_2, n):
	var closest_point_idx = []
	
	var distance_buffer = []
	
	for p2 in pc_2:
		distance_buffer = []
		for p1 in pc_1:
			distance_buffer.append((p2 - p1).length())
		
		closest_point_idx.append(min_index(distance_buffer))
		
	print(closest_point_idx)

func min_index(arr):
	var min = INF
	var index = 0
	
	for i in arr.size():
		if arr[i] < min:
			min = arr[i]
			index = i
	
	return index
