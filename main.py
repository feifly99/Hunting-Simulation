import matplotlib.pyplot as plt
import numpy as np

def parse_data(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        steps = f.read().strip().split('QAQ')

    parsed_data = []
    for step in steps:
        hunters = []
        obstacles = []
        lines = step.strip().split('\n')
        for line in lines:
            if line.startswith('hunter'):
                parts = line.split(', ')
                x = float(parts[0].split('=')[1].strip())
                y = float(parts[1].split('=')[1].strip())
                hunters.append((x, y))
            elif line.startswith('target'):
                target = (float(line.split('=')[1].split(',')[0].strip()),
                          float(line.split('=')[2].strip()))
            elif line.startswith('obstacle'):
                parts = line.split(', ')
                x = float(parts[0].split('=')[1].strip())
                y = float(parts[1].split('=')[1].strip())
                obstacles.append((x, y))
        parsed_data.append((hunters, target, obstacles))
    return parsed_data

def plot_data(data, radius=20, gap=50):
    plt.ion()
    fig, ax = plt.subplots()

    # 获取初始坐标范围
    all_x = []
    all_y = []
    for hunters, target, obstacles in data:
        all_x.extend([x for x, _ in hunters] + [target[0]] + [x for x, _ in obstacles])
        all_y.extend([y for _, y in hunters] + [target[1]] + [y for _, y in obstacles])

    min_x, max_x = min(all_x) - radius, max(all_x) + radius
    min_y, max_y = min(all_y) - radius, max(all_y) + radius

    # 初始设置一次，后续不动
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_title('Hunter Simulation')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Hunting Simulation Viewer')

    cmap = plt.get_cmap('tab10')

    for step, (hunters, target, obstacles) in enumerate(data):
        ax.clear()
        ax.set_title(f'Step {step + 1}')
        ax.grid(True)
        ax.set_aspect('equal', adjustable='datalim')  # 保证圆形不被拉伸！

        # 绘制猎人
        for i, (x, y) in enumerate(hunters):
            ax.scatter(x, y, c=[cmap(i % 10)], label=f'Hunter {i}', s=50)

        # 绘制目标
        ax.scatter(*target, c='red', marker='*', s=150, label='Target')

        # 绘制障碍物
        if obstacles:
            ox, oy = zip(*obstacles)
            ax.scatter(ox, oy, c='green', marker='x', s=50, label='Obstacles')
            for x, y in obstacles:
                circle = plt.Circle((x, y), radius, color='green', fill=False, linewidth=1.5, linestyle='--')
                ax.add_patch(circle)

        ax.legend(loc='upper right')
        plt.pause(gap / 1000.0)

    plt.ioff()
    plt.show()


# 示例用法
file_path = 'D:/VS PROJECTS/HuntingSimulation/HuntingSimulation/simulationData.txt'  # 注意：用正斜杠更稳定
data = parse_data(file_path)
plot_data(data, radius=6, gap=20)
