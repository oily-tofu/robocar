#!/usr/bin/env python3
from collections import deque
import rospy
import random
from way_r.msg import way_def, way_save

rows, cols = 4, 3
start = (3, 1)
dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

fake_l0 = []
real_l2 = []
real_l1 = []

def callback(msg):
    global fake_l0,real_l1,real_l2
    fake_l0.clear()
    real_l1.clear()
    real_l2.clear()
    for idx, point in enumerate(msg.way):
        x = point.x
        y = point.y
        z = point.z
        t = point.timestamp
        if z == 0:
            fake_l0.append((x,y))
        if z == 1:
            real_l1.append((x,y))
        if z == 2:
            real_l2.append((x,y))
        rospy.loginfo("Point %d: x=%f, y=%f, z=%f, timestamp=%s", idx, x, y, z, t)

    path_a = bfs(start, real_l2)
    write(path_a,real_l2,real_l1)
    path_a.clear()

def subscriber():

    rospy.init_node('sub_kfs_node', anonymous=True)
    rospy.Subscriber('kfs', way_save, callback)
    rospy.spin()


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
                        (nx, ny) not in fake_l0 and
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
            elif (i, j) in fake_l0:
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
    subscriber()
    # path_a = bfs(start, real_l2)
    # write(path_a,real_l2,real_l1)


# (0,0) (0,1) (0,2)
# (1,0) (1,1) (1,2)
# (2,0) (2,1) (2,2)
# (3,0) (3,1) (3,2)

    

