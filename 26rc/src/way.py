from collections import deque

# 地图大小
rows, cols = 4, 3

# 起点、终点坐标
start = (3, 1)  # 底部中间
goal = (8, 8)  # 右上角

# 四个方向移动：上，下，左，右
dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def bfs(start, goal):
    queue = deque([[start]])
    visited = set([start])

    while queue:
        path = queue.popleft()
        x, y = path[-1]

        # 到达目标
        if (x, y) == goal:
            return path

        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append(path + [(nx, ny)])
    return None


path = bfs(start, goal)

print("最短路径：")
for step in path:
    print(step)


    # (0,0) (0,1) (0,2)
    # (1,0) (1,1) (1,2)
    # (2,0) (2,1) (2,2)
    # (3,0) (3,1) (3,2)