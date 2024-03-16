import scipy.io as scio
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import math

hz = 120
t_takeoff = 3600 / hz
is_debug = True

# Ref
# https://blog.csdn.net/clksjx/article/details/105720120
# https://blog.csdn.net/qq_40985985/article/details/119676953
# https://blog.csdn.net/weixin_44517301/article/details/102890334
# https://zhuanlan.zhihu.com/p/442932579
# https://blog.csdn.net/u014636245/article/details/82799573

class vicon_MAS:

    def __init__(self, hz = 120):
        self.xyz  = list()           # list of XYZ
        self.Vxyz = list()           # list of XYZ
        self.relative_dist = list()
        self.t    = list()
        self.n_num = 0

        self.hz = hz                # 根据这个来差值

    def align_data(self, MAS_xyz_all, MAS_t_all, hz = 0):

        self.n_num = MAS_xyz_all.__len__()
        data_len_i = []
        for i in range(0, self.n_num):
            size_i = MAS_xyz_all[i].__len__()
            data_len_i.append(size_i)
        data_len = max(data_len_i)

        x_cluster = []
        y_cluster = []
        z_cluster = []
        t_cluster = []
        for i in range(0, data_len):
            x_cluster_t = []
            y_cluster_t = []
            z_cluster_t = []
            t_cluster_t = []
            for j in range(0, self.n_num):
                if i < MAS_xyz_all[j].__len__():
                    x_t = MAS_xyz_all[j][i][0]
                    y_t = MAS_xyz_all[j][i][1]
                    z_t = MAS_xyz_all[j][i][2]
                    t_t = MAS_t_all[j][i]
                else:
                    x_t = MAS_xyz_all[j][MAS_xyz_all[j].__len__() - 1][0]
                    y_t = MAS_xyz_all[j][MAS_xyz_all[j].__len__() - 1][1]
                    z_t = MAS_xyz_all[j][MAS_xyz_all[j].__len__() - 1][2]
                    t_t = MAS_t_all[j][MAS_t_all[j].__len__() - 1]                 # 理论上MAS_t_all[j]和MAS_xyz_all[j]的长度应该是一样的
                #
                x_cluster_t.append(x_t)
                y_cluster_t.append(y_t)
                z_cluster_t.append(z_t)
                t_cluster_t.append(t_t)

            x_cluster.append(x_cluster_t)
            y_cluster.append(y_cluster_t)
            z_cluster.append(z_cluster_t)
            t_cluster.append(t_cluster_t)

        self.align_data_distributed(x_cluster, y_cluster, z_cluster, t_cluster)

    def align_data_distributed(self, x_cluster_in, y_cluster_in, z_cluster_in, t_cluster_in, hz = 0):

        self.n_num = x_cluster_in[0].__len__()
        data_len  = x_cluster_in.__len__()
        if hz == 0:
            hz = self.hz
        dt = 1. / hz

        for i in range(0, data_len - 1):
            xyz_i = list()
            Vxyz_i = list()
            for j in range(0, self.n_num):
                #
                t_now = dt * i
                t_j = t_cluster_in[i][j]
                #
                x_now = x_cluster_in[i][j]
                y_now = y_cluster_in[i][j]
                z_now = z_cluster_in[i][j]
                #
                x_next = x_cluster_in[i + 1][j]
                y_next = y_cluster_in[i + 1][j]
                z_next = z_cluster_in[i + 1][j]
                #
                vx_j = (x_next - x_now) / dt
                vy_j = (y_next - y_now) / dt
                vz_j = (z_next - z_now) / dt
                #
                # Method 1
                # if (t_j < t_now):         # 后面发现时间最好不要动
                #     t_now += dt
                # x_comp = x_now + vx_j * (t_now - t_j)
                # y_comp = y_now + vy_j * (t_now - t_j)
                # z_comp = z_now + vz_j * (t_now - t_j)
                # Method 2
                # if (t_now > t_j):
                #     x_comp = x_now + vx_j * (t_now - t_j)
                #     y_comp = y_now + vy_j * (t_now - t_j)
                #     z_comp = z_now + vz_j * (t_now - t_j)
                # else:
                #     x_comp = x_now + vx_j * (t_j - t_now)
                #     y_comp = y_now + vy_j * (t_j - t_now)
                #     z_comp = z_now + vz_j * (t_j - t_now)
                # Method 3
                # No compension
                x_comp = x_now
                y_comp = y_now
                z_comp = z_now

                #
                xyz_t = [x_comp, y_comp, z_comp]
                Vxyz_t = [vx_j, vy_j, vz_j]

                xyz_i.append(xyz_t)
                Vxyz_i.append(Vxyz_t)
            self.xyz.append(xyz_i)
            self.Vxyz.append(Vxyz_i)
            self.t.append(t_now)

        return [self.xyz, self.Vxyz, self.t]

    def get_relative_dist(self):
        for tx in range(0, self.t.__len__()):
            dist_tx = dict()
            for i in range(0, self.n_num):
                for j in range(0, self.n_num):
                    dx = self.xyz[tx][i][0] - self.xyz[tx][j][0]
                    dy = self.xyz[tx][i][1] - self.xyz[tx][j][1]
                    dz = self.xyz[tx][i][2] - self.xyz[tx][j][2]

                    dist_t = math.sqrt(dx**2 + dy**2 + dz**2)
                    key_name = str(i) + "-" + str(j)
                    dist_tx[key_name] = dist_t

            self.relative_dist.append(dist_tx)

        return self.relative_dist

    def get_data_seprated(self):
        data_len = self.xyz.__len__()
        data_sep = []
        for i in range(0, self.n_num):
            x_i = []
            y_i = []
            z_i = []
            for j in range(0, data_len):
                x_t = self.xyz[j][i][0]
                y_t = self.xyz[j][i][1]
                z_t = self.xyz[j][i][2]

                x_i.append(x_t)
                y_i.append(y_t)
                z_i.append(z_t)

            agent_i_data = [x_i, y_i, z_i]
            data_sep.append(agent_i_data)

        return data_sep

def get_t_data_len(data_t):
    t_start = float(data_t[0])
    data_len = list(data_t).__len__()
    t = list()
    for i in range(0, data_len):
        t_i = float(data_t[i]) - t_start
        t.append(t_i)

    return [t, data_len]

def get_target_pos(x_t, y_t, z_t, data_len):

    target_pos = list()

    for i in range(0, data_len):
        x = float(x_t[i])
        y = float(y_t[i])
        z = float(z_t[i])

        target_pos.append([x, y, z])

    return target_pos


def update(num, data, lines):
    for i in range(0, lines.__len__() - 1):
        lines[i].set_data([data[i][0][0:num], data[i][1][0:num]])
        lines[i].set_3d_properties(data[i][2][0:num], 'z')
        print(num)
    return lines

def main():
    global t_takeoff, hz, is_debug

    data = scio.loadmat('./data/uav_data20240315T202654.mat')
    print("loaded")

    #
    [uav1_t, uav1_data_len] = get_t_data_len(data['uav1_pos_t'])
    uav1_xyz = get_target_pos(data['uav1_x'], data['uav1_y'], data['uav1_z'], uav1_data_len)
    #
    [uav2_t, uav2_data_len] = get_t_data_len(data['uav2_pos_t'])
    uav2_xyz = get_target_pos(data['uav2_x'], data['uav2_y'], data['uav2_z'], uav2_data_len)
    #
    [leader_t, leader_data_len] = get_t_data_len(data['uav3_pos_t'])
    leader_xyz = get_target_pos(data['uav3_x'], data['uav3_y'], data['uav3_z'], leader_data_len)         #
    #
    [target_t, target_data_len] = get_t_data_len(data['target_pos_t'])
    target_xyz = get_target_pos(data['target_x'], data['target_y'], data['target_z'], target_data_len)

    mas = vicon_MAS(hz)

    mas.align_data([uav1_xyz, uav2_xyz, leader_xyz, target_xyz], [uav1_t, uav2_t, leader_t, target_t], hz=hz)
    mas.get_relative_dist()
    xyz  = mas.xyz
    Vxyz = mas.Vxyz
    relative_dist = mas.relative_dist
    t    = mas.t

    xyz_seprated = mas.get_data_seprated()
    print("Data obatained, plotting ...")

    if is_debug:
        i_end = 1200
    else:
        i_end = t.__len__()

    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    # https://blog.csdn.net/u014636245/article/details/82799573
    #lines = []
    #for i in range(0, mas.n_num):
    #    line_t, = ax.plot3D(xyz_seprated[i][0], xyz_seprated[i][1], xyz_seprated[i][2])
    #    lines.append(line_t)
    #
    lines = [ax.plot3D(dat[0], dat[1], dat[2])[0] for dat in xyz_seprated]

    # https://zhuanlan.zhihu.com/p/442932579
    ani = animation.FuncAnimation(fig, update, frames=len(xyz_seprated[0][0]), fargs=(xyz_seprated, lines),
                            interval=1000./ hz, blit=False)

    plt.show()

    # Writer = animation.writers['ffmpeg']  # 需安装ffmpeg
    # writer = Writer(fps=hz, metadata=dict(artist='Me'), bitrate=1800)
    # animation.save("movie.mp4", writer=writer)

    #ani.save("3.gif", fps=25, writer="imagemagick")


    print(233)


if __name__ ==  '__main__':
    main()
