import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'site-packages'))
print(os.path.join(os.path.dirname(__file__)))

import casadi as ca
import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import PathWithLaneId, Trajectory, TrajectoryPoint

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.animation as ani


df = pd.read_csv('/aichallenge/workspace/src/aichallenge_submit/path_to_trajectory/path_to_trajectory/nodes.csv')
# /home/taka/jaaic/aichallenge-2024/aichallenge/workspace/src/aichallenge_submit/path_to_trajectory/path_to_trajectory/nodes.csv

# データの先頭を表示して確認
print(df.head())


# 中央線、左壁、右壁の情報を抽出
centerline_coords = df[df['Wall Info'] == 'Centerline'][['X (meters)', 'Y (meters)']].values
left_wall_coords = df[df['Wall Info'] == 'Left'][['X (meters)', 'Y (meters)']].values
right_wall_coords = df[df['Wall Info'] == 'Right'][['X (meters)', 'Y (meters)']].values

centerline_coords = centerline_coords[1:]
centerline_coords = centerline_coords[::2]
# left_wall_coords = left_wall_coords[:30]  # 10mごとにサンプリング
# right_wall_coords = right_wall_coords[:30]  # 10mごとにサンプリ

init_points = [[p[0], p[1]] for p in centerline_coords]


class Obstacle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
    def subject_to_obstacle(self,opti):
        for i in range(N):
            opti.subject_to((X[0,i] - self.x)**2 + (X[1,i] - self.y)**2 >= self.r **2)
class Wall:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
    def subject_to_wall(self,opti):
        for i in range(N):
            opti.subject_to((X[0,i] - self.x1)*(self.y2 - self.y1) - (X[1,i] - self.y1)*(self.x2 - self.x1) >= 0)

class Checkpoint:
    def __init__(self, x_range, y_range):
        global init_points
        self.x_range = x_range
        self.y_range = y_range
        self.node_id = -1
        max_dist = 0
        for i in range(len(init_points)):
            p = init_points[i]
            if p[0] < x_range[0] or p[0] > x_range[1] or p[1] < y_range[0] or p[1] > y_range[1]:
                continue
            dist = min(abs(p[0]-x_range[0]), abs(p[0]-x_range[1]), abs(p[1]-y_range[0]), abs(p[1]-y_range[1]))
            if dist > max_dist:
                max_dist = dist
                self.node_id = i
    def subject_to_checkpoint(self, opti, X):
        opti.subject_to(X[0,self.node_id] >= self.x_range[0])
        opti.subject_to(X[0,self.node_id] <= self.x_range[1])
        opti.subject_to(X[1,self.node_id] >= self.y_range[0])
        opti.subject_to(X[1,self.node_id] <= self.y_range[1])

def create_points(init_points):

    # Optiオブジェクトの作成
    opti = ca.Opti()

    # 定数設定
    MAX_ACCEL = 3.2  # 最大加速度 [m/s^2]
    MAX_SPEED = 8.3333  # 最大速度 [m/s]
    MIN_DISTANCE_TO_WALL = 3.0  # 壁からの最小距離 [m]
    N = 0  # 経由点の数（後で設定）
    dt = 0.1  # タイムステップ

    # CSVから地図データを読み込む
    # df = pd.read_csv('specific_relation_nodes_coordinates_with_wall_info2.csv')


    obstacles = [Obstacle(11.4, 38.1 + 7.2, 2),
                Obstacle(41.4, 18.8 + 7.2, 2.5),
                 Obstacle(19.2, 47.7 + 7.2, 1.3),
                Obstacle(20, 56.6 + 7.2, 2.8),
                Obstacle(25.6, 60 + 7.2, 2.3),
                Obstacle(40.6, 65.1 + 7.2, 5),
                Obstacle(60.6, 58.8 + 7.2, 2.5),
                Obstacle(50.3, 45.2 + 7.2, 2),
                Obstacle(61.4, 42 + 7.2, 2.4),
                Obstacle(74.6, 43 + 7.2, 3),
                Obstacle(82.1, 33.8 + 7.2, 2.4),
                Obstacle(74.5, 38.7, 3.5),
                Obstacle(65.2, 18.5 + 7.2, 2.8),
                Obstacle(50.3, 12.6, 4)]


    # checkpoints = [Checkpoint(0, [30, 35], [0, 10]),
    #                 Checkpoint(17, [2, 11], [38, 48]),
    #                 Checkpoint(29, [41, 51], [9, 18]),
    #                 Checkpoint(45, [10, 20], [55, 59]),
    #                 Checkpoint(50, [25, 40], [60, 65]),
    #                 Checkpoint(62, [60, 70], [58, 64]),
    #                 Checkpoint(70, [47, 51], [46, 49]),
    #                 Checkpoint(72, [47, 51], [41, 45]),
    #                 Checkpoint(76, [58, 62], [35, 41]),
    #                 Checkpoint(80, [70, 78], [45, 50]),
    #                 Checkpoint(85, [83, 95], [30, 40]),
    #                 Checkpoint(92, [58, 65], [20, 25])]
    checkpoints = [Checkpoint([22, 35], [0 + 7.2, 10 + 7.2]),
                    Checkpoint([0, 11], [34 + 7.2, 41 + 7.2]),
                    Checkpoint([8, 13], [40 + 7.2, 50 + 7.2]),
                    Checkpoint([41, 51], [9 + 7.2, 18 + 7.2]),
                    Checkpoint([18, 30], [49 + 7.2, 55 + 7.2]),
                    Checkpoint([25, 35], [40 + 7.2, 45 + 7.2]),
                    Checkpoint([7, 18], [51 + 7.2, 60 + 7.2]),
                    Checkpoint([15, 22], [60 + 7.2, 70 + 7.2]),
                    Checkpoint([25, 40], [57 + 7.2, 65 + 7.2]),
                    Checkpoint([60, 70], [58 + 7.2, 67 + 7.2]),
                    Checkpoint([50, 63], [48 + 7.2, 55 + 7.2]),
                    Checkpoint([40, 49], [40 + 7.2, 50 + 7.2]),
                    # Checkpoint([47, 51], [41, 45]),
                    Checkpoint([55, 62], [30 + 7.2, 41 + 7.2]),
                    Checkpoint([70, 78], [45 + 7.2, 53 + 7.2]),
                    Checkpoint([83, 95], [30 + 7.2, 40 + 7.2]),
                    Checkpoint([58, 65], [19 + 7.2, 27 + 7.2]),
                    Checkpoint([40, 60], [0, 10])]

    # #デバッグ用マップ描画
    # plt.figure(figsize=(10, 10))
    # plt.scatter([p[0] for p in init_points], [p[1] for p in init_points], c='blue', marker='o', edgecolor='k')
    # plt.scatter(left_wall_coords[:, 0], left_wall_coords[:, 1], c='red', marker='o', edgecolor='k')
    # plt.scatter(right_wall_coords[:, 0], right_wall_coords[:, 1], c='green', marker='o', edgecolor='k')
    # for p in obstacles:
    #     plt.gca().add_patch(
    #         patch.Circle((p.x, p.y), p.r, edgecolor="black", facecolor="r", alpha=0.8)
    #     )
    # cp = []
    # for p in checkpoints:
    #     plt.gca().add_patch(
    #         patch.Rectangle((p.x_range[0], p.y_range[0]), p.x_range[1]-p.x_range[0], p.y_range[1]-p.y_range[0], edgecolor="black", facecolor="g", alpha=0.8)
    #     )
    #     cp.append(init_points[p.node_id])

    # plt.scatter([p[0] for p in cp], [p[1] for p in cp], c='black', marker='x', edgecolor='k')

    # plt.xlabel('X (meters)')
    # plt.ylabel('Y (meters)')
    # plt.title('OSM Nodes for Specific Relation in Meter Coordinate System')
    # plt.gca().set_aspect('equal')
    # plt.grid(True)
    # plt.show()

    # 経由点の数を中央線のノード数に設定
    N = len(init_points)-1

    print("N:", N)

    # 状態変数と制御入力
    X = opti.variable(6, N+1)  # X = [[x, y, th, vx, vy, vth], ...]
    U = opti.variable(3, N)  # U = [[ux, uy, uth], ...]
    T = opti.variable(N+1)  # T は各経由点での時刻

    T_total = ca.sum1(T)

    # Logの準備
    log = []
    opti.callback(lambda i: log.append((i, opti.debug)))  # logの追加

    # 制約条件
    # 運動方程式に従う
    def subject_to_dynamics():
        dt = T_total / N
        for i in range(N):
            # dt = T[i+1] - T[i]  
            # x_next = ca.vertcat(
            #     X[0,i] + dt * X[3,i] + 0.5 * dt**2 * U[0,i],  # x
            #     X[1,i] + dt * X[4,i] + 0.5 * dt**2 * U[1,i],  # y
            #     X[2,i] + dt * X[5,i] + 0.5 * dt**2 * U[2,i],  # th
            #     X[3,i] + dt * U[0,i],  # vx
            #     X[4,i] + dt * U[1,i],  # vy
            #     X[5,i] + dt * U[2,i]   # vth
            # )
            # opti.subject_to(X[:,i+1] == x_next)
            opti.subject_to((X[0,i+1] - X[0,i]) ** 2 + (X[1,i+1] - X[1,i]) ** 2 <= (MAX_SPEED* dt)**2)

    # 壁制約
    nearest_obstacle = [[] for i in range(N+1)]
    for i in range(N+1):
        for p in obstacles:
            if (init_points[i][0] - p.x)**2 + (init_points[i][1] - p.y)**2 < (10 + p.r)**2:
                nearest_obstacle[i].append(p)

    def subject_to_wall():
        for i in range(N):
            for o in nearest_obstacle[i]:
                opti.subject_to((X[0,i] - o.x)**2 + (X[1,i] - o.y)**2 >= (o.r + (1.45 / 2)) **2)
            # for p in left_wall_coords:
            #     opti.subject_to((X[0,i] - p[0])**2 + (X[1,i] - p[1])**2 >= MIN_DISTANCE_TO_WALL**2)
            # for p in right_wall_coords:
            #     opti.subject_to((X[0,i] - p[0])**2 + (X[1,i] - p[1])**2 >= MIN_DISTANCE_TO_WALL**2)



    def subject_to_checkpoints():
        for c in checkpoints:
            c.subject_to_checkpoint(opti,X)

    # 障害物制約
    # 障害物の中心と半径がリストで[x, y, r]と与えられたとき、半径+1m以上離れる
    def subject_to_obstacles(L):
        for i in range(N):
            for p in L:
                opti.subject_to((X[0,i] - p[0])**2 + (X[1,i] - p[1])**2 >= (p[2] + 1)**2)

    # 加速度制約
    def subject_to_acc_limit():
        for i in range(N):
            opti.subject_to(ca.fabs(U[0,i])**2 + ca.fabs(U[1,i])**2 <= MAX_ACCEL**2)

    subject_to_dynamics()
    subject_to_wall()
    subject_to_checkpoints()
    subject_to_acc_limit()

    # 終端拘束
    # 始点は中央線の最初の点で停止している
    opti.subject_to(X[0,0] == init_points[0][0])
    opti.subject_to(X[1,0] == init_points[0][1])
    opti.subject_to(X[2,0] == 0)  # th
    opti.subject_to(X[3,0] == 0)  # vx
    opti.subject_to(X[4,0] == 0)  # vy
    opti.subject_to(X[5,0] == 0)  # vth
    opti.subject_to(U[0,0] == 0)  # acc_x
    opti.subject_to(U[1,0] == 0)  # acc_y
    opti.subject_to(U[2,0] == 0)  # acc_th

    # 終端拘束
    # 終点は中央線の最後の点で停止している
    opti.subject_to(X[0,N] == init_points[-1][0])
    opti.subject_to(X[1,N] == init_points[-1][1])
    opti.subject_to(X[2,N] == 0)  # th
    opti.subject_to(X[3,N] == 0)  # vx
    opti.subject_to(X[4,N] == 0)  # vy
    opti.subject_to(X[5,N] == 0)  # vth
    opti.subject_to(U[0,N-1] == 0)  # acc_x
    opti.subject_to(U[1,N-1] == 0)  # acc_y
    opti.subject_to(U[2,N-1] == 0)  # acc_th

    # 目的関数
    # 時間の最小化
    opti.minimize(T_total)

    # 初期解の設定
    # 状態変数Xの初期解を中央線の座標で設定
    init_x = np.zeros((6, N+1))
    for i in range(N+1):
        init_x[0, i] = init_points[i][0]
        init_x[1, i] = init_points[i][1]
        init_x[4, i] = 0
        if i != N:
            init_x[3, i] = (init_points[i+1][0] - init_points[i][0]) / (40.0 / N)
            init_x[4, i] = (init_points[i+1][1] - init_points[i][1]) / (40.0 / N)
        init_x[5, i] = 0


    # 制御入力Uの初期解を0で設定
    init_u = np.zeros((3, N))

    # 各経由点での時刻の初期解
    init_T = np.linspace(0, 40, N+1)

    # 初期解をOptiに設定
    opti.set_initial(X, init_x)
    opti.set_initial(U, init_u)
    opti.set_initial(T, init_T)

    #障害物の中心と半径
    obstacle = [[50, 50, 5]]

    # 最適化問題を解く
    opti.solver("ipopt")
    try:
        opti.solve()
    except:
        print("error")

    # #解の描画
    # plt.plot(opti.value(X[0,:]), opti.value(X[1,:]))
    # # plt.gca().add_patch(
    # #     patch.Circle((50, 50), 5, edgecolor="black", facecolor="r", alpha=0.8)
    # # )
    # plt.xlim(0,100)
    # plt.ylim(0,100)
    # for p in obstacles:
    #     plt.gca().add_patch(
    #         patch.Circle((p.x, p.y), p.r, edgecolor="black", facecolor="r", alpha=0.8)
    #     )
    # cp = []
    # for p in checkpoints:
    #     plt.gca().add_patch(
    #         patch.Rectangle((p.x_range[0], p.y_range[0]), p.x_range[1]-p.x_range[0], p.y_range[1]-p.y_range[0], edgecolor="black", facecolor="g", alpha=0.8)
    #     )
    #     cp.append([opti.value(X[0,p.node_id]),opti.value(X[1,p.node_id])])
    # plt.scatter([p[0] for p in cp], [p[1] for p in cp], c='black', marker='x', edgecolor='k')
    # # plt.plot(left_wall_coords[:, 0], left_wall_coords[:, 1], c='red')
    # # plt.plot(right_wall_coords[:, 0], right_wall_coords[:, 1], c='green')
    # plt.scatter(left_wall_coords[:, 0], left_wall_coords[:, 1], c='red', marker='o', edgecolor='k', s=2)
    # plt.scatter(right_wall_coords[:, 0], right_wall_coords[:, 1], c='green', marker='o', edgecolor='k', s=2)
    # plt.show()

    # print("log:",log)
    # #最適化のlogの描画

    # def draw_frame(i):
    #     j, debug = log[i]
    #     plt.cla()
    #     # plt.gca().add_patch(
    #     #     patch.Circle((50, 50), 5, edgecolor="black", facecolor="r", alpha=0.8)
    #     # )
    #     plt.scatter(debug.value(X[0,:]), debug.value(X[1,:]))
    #     plt.plot(debug.value(X[0,:]), debug.value(X[1,:]))
    #     plt.xlim(0, 100)
    #     plt.ylim(0, 100)
    #     for p in obstacles:
    #         plt.gca().add_patch(
    #             patch.Circle((p.x, p.y), p.r, edgecolor="black", facecolor="r", alpha=0.8)
    #         )
    #     for p in checkpoints:
    #         plt.gca().add_patch(
    #             patch.Rectangle((p.x_range[0], p.y_range[0]), p.x_range[1]-p.x_range[0], p.y_range[1]-p.y_range[0], edgecolor="black", facecolor="g", alpha=0.8)
    #         )
    #     plt.scatter([p[0] for p in cp], [p[1] for p in cp], c='black', marker='x', edgecolor='k')

    # fig = plt.figure()
    # _ = ani.FuncAnimation(fig, draw_frame, frames=len(log), interval=10)
    # plt.show()

    return [[opti.value(X[0,i]), opti.value(X[1,i])] for i in range(N+1)]

# if __name__ == '__main__':
#     next_points = main(init_points)
#     for i in range(5):
#         next_points = main(next_points)



class PathToTrajectory(Node):

    def __init__(self):
        super().__init__('path_to_trajectory_node')
        self.pub_ = self.create_publisher(Trajectory, 'output', 10)
        self.sub_ = self.create_subscription(
            PathWithLaneId,
            'input',
            self.callback,
            10
        )
        self.got_new_obstacle = True
        self.result = []

    def callback(self, msg: PathWithLaneId):
        trajectory = Trajectory()
        trajectory.header = msg.header
        if self.got_new_obstacle:
            self.result = create_points(init_points)
            self.got_new_obstacle = False
        # print(msg)
        # for path_point_with_lane_id in msg.points:
        #     trajectory_point = TrajectoryPoint()
        #     trajectory_point.pose = path_point_with_lane_id.point.pose
        #     trajectory_point.longitudinal_velocity_mps = path_point_with_lane_id.point.longitudinal_velocity_mps
        #     trajectory.points.append(trajectory_point)
        def to_latelong(x, y):
            # return x + msg.points[0].point.pose.position.x, y + msg.points[0].point.pose.position.y
            return (x + 15560341.360989474) / (np.pi / 180 * 6378137), (y + 3965832.877402093) / (np.pi / 180 * 6378137)
        for p in self.result:
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose.position.x, trajectory_point.pose.position.y = to_latelong(p[0], p[1])
            trajectory_point.longitudinal_velocity_mps = 30 * 1000 / 3600
            trajectory.points.append(trajectory_point)
        self.pub_.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = PathToTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

