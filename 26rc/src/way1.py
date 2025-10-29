from collections import deque
import random

rows, cols = 4, 3
start = (3, 1)
dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

#生成随机假 KFS
x_fake = random.randint(0, 2)
y_fake = random.randint(0, 2)
obstacles = {(x_fake, y_fake)}

real_l2 = []
real_l1 = []

#生成随机r2 KFS
def real_r2(real_l2):
    real_i = 0
    while(1):
        x = random.randint(0, 3)
        y = random.randint(0, 2)
        if (x, y) not in obstacles and (x, y) not in real_l2:
            real_l2.append((x, y))
            real_i += 1
        if real_i == 4:
            break
    return real_l2

#生成随机r1 KFS
def real_r1(real_l1,real_l2):
    real_i = 0
    while(1):
        x = random.randint(0, 3)
        y = random.randint(0, 2)
        if (x, y) not in obstacles and (x, y) not in real_l2 and (x, y) not in real_l1 and (x, y) not in [(2,1),(1,1)]:
            real_l1.append((x, y))
            real_i += 1
        if real_i == 3:
            break
    return real_l1

#广度优先
def bfs(start_1, goal_2):
    path_list = []

    for i, goal_1 in enumerate(goal_2):
        queue = deque([[start_1]])
        visited = set([start_1])
        found_path = None

        while queue:
            path = queue.popleft()
            x, y = path[-1]

            if (x, y) == goal_1:
                found_path = {
                    'path_num': i,  # 路径编号
                    'path': path,  # 实际路径
                    'goal': goal_1,  # 目标点
                    'length': len(path)  # 路径长度
                }
                break

            for dx, dy in dirs:
                nx, ny = x + dx, y + dy
                if (0 <= nx < rows and 0 <= ny < cols and
                        (nx, ny) not in obstacles and
                        (nx, ny) not in visited and
                        (nx, ny) not in real_l1):
                    visited.add((nx, ny))
                    queue.append(path + [(nx, ny)])

        if found_path:
            path_list.append(found_path)

    path_list.sort(key=lambda x: x['length'])

    return path_list



def write(path_1,real_l2,real_l1):
    print("地图布局：")
    for i in range(rows):
        for j in range(cols):
            if (i, j) in real_l2:
                print("( 2 )", end=" ")  # 终点
            elif (i, j) in obstacles:
                print("( X )", end=" ")  # 路障
            elif (i, j) in real_l1:
                print("( 1 )", end=" ")  # 路障
            else:
                print(f"({i},{j})", end=" ")
        print()

    print("\n最短路径：")
    if path_1:
        for step in path_1:
            print(step)
    else:
        print("无法到达目标点！")


if __name__ == "__main__":
    real_l2 = real_r2(real_l2)
    real_l1 = real_r1(real_l1,real_l2)
    path_a = bfs(start, real_l2)
    write(path_a,real_l2,real_l1)


# (0,0) (0,1) (0,2)
# (1,0) (1,1) (1,2)
# (2,0) (2,1) (2,2)
# (3,0) (3,1) (3,2)