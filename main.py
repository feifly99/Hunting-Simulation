import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 文件路径
file_path = r"E:\VS SOLUTIONS\mathworks\mathworks\output.txt"

# 读取文件数据
def read_data(file_path):
    with open(file_path, 'r') as f:
        data = f.readlines()

    hunter_positions = [[], [], []]
    target_positions = []

    for i in range(0, len(data), 5):  # 每5行一个循环
        # 提取捕猎者和目标的坐标
        for j in range(3):
            line = data[i + j]
            parts = line.split(',')
            x = float(parts[0].split('=')[1].strip())
            y = float(parts[1].split('=')[1].strip())
            hunter_positions[j].append((x, y))

        # 提取目标坐标
        target_line = data[i + 3]
        tx = float(target_line.split(',')[0].split('=')[1].strip())
        ty = float(target_line.split(',')[1].split('=')[1].strip())
        target_positions.append((tx, ty))

    return hunter_positions, target_positions

# 绘图更新函数
def update(num, hunter_positions, target_positions, scatters, lines):
    for i in range(3):
        # 更新捕猎者位置和轨迹
        scatters[i].set_offsets([hunter_positions[i][num]])
        lines[i].set_data([p[0] for p in hunter_positions[i][:num + 1]],
                          [p[1] for p in hunter_positions[i][:num + 1]])

    # 更新目标位置
    scatters[3].set_offsets([target_positions[num]])

# 读取数据
hunter_positions, target_positions = read_data(file_path)

# 创建图形
fig, ax = plt.subplots()
ax.set_xlim(-20*4*3, 10*4*3)  # 设置坐标轴范围，可以根据数据调整
ax.set_ylim(-10*4*3, 18*4*3)

# 设置坐标轴比例相同
ax.set_aspect('equal', adjustable='box')

# 初始化散点和轨迹线条
shapes = ['o', 's', '^']  # 不同捕猎者的形状
colors = ['blue', 'green', 'purple']
scatters = [ax.scatter([], [], color=colors[i], marker=shapes[i], s=100, label=f'Hunter {i}') for i in range(3)]
scatters.append(ax.scatter([], [], color='red', marker='*', s=120, label='Target'))  # 目标点
lines = [ax.plot([], [], color=colors[i])[0] for i in range(3)]

# 添加图例
ax.legend()

# 动态更新绘图
ani = animation.FuncAnimation(fig, update, frames=len(target_positions),
                              fargs=(hunter_positions, target_positions, scatters, lines),
                              interval=750, repeat=False)

# 显示动画
plt.show()
