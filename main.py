import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.cm as cm

if __name__ == "__main__":

    # File path
    file_path = r"E:\VS SOLUTIONS\mathworks\mathworks\output.txt"

    # Number of hunters
    huntersNum = 5
    gap = 1000
    x_min = -50 * 3
    y_min = -50 * 3
    x_max = 50 * 3
    y_max = 50 * 3
    # Function to read data from file
    def read_data(file_path):
        with open(file_path, 'r') as f:
            data = f.readlines()

        hunter_positions = [[] for _ in range(huntersNum)]
        target_positions = []

        # Process data, each cycle consists of huntersNum lines for hunters, 1 line for target, 1 line for QAQ
        for i in range(0, len(data), huntersNum + 2):
            # Extract hunter positions
            for j in range(huntersNum):
                line = data[i + j]
                parts = line.split(',')
                x = float(parts[0].split('=')[1].strip())
                y = float(parts[1].split('=')[1].strip())
                hunter_positions[j].append((x, y))

            # Extract target position
            target_line = data[i + huntersNum]
            tx = float(target_line.split(',')[0].split('=')[1].strip())
            ty = float(target_line.split(',')[1].split('=')[1].strip())
            target_positions.append((tx, ty))

        return hunter_positions, target_positions

    # Update function for animation
    def update(num, hunter_positions, target_positions, scatters, lines):
        for i in range(huntersNum):
            # Update hunter positions and paths
            scatters[i].set_offsets([hunter_positions[i][num]])
            lines[i].set_data([p[0] for p in hunter_positions[i][:num + 1]],
                              [p[1] for p in hunter_positions[i][:num + 1]])

        # Update target position and path
        scatters[huntersNum].set_offsets([target_positions[num]])
        lines[huntersNum].set_data([p[0] for p in target_positions[:num + 1]], [p[1] for p in target_positions[:num + 1]])

    # Read data
    hunter_positions, target_positions = read_data(file_path)

    # Create figure
    fig, ax = plt.subplots()
    ax.set_xlim(x_min, x_max)  # Adjusted as needed
    ax.set_ylim(y_min, y_max)

    # Ensure equal aspect ratio
    ax.set_aspect('equal', adjustable='box')

    # Initialize scatter plots and lines with circles for hunters
    colors = cm.get_cmap('tab10', huntersNum)  # Generates distinct colors for each hunter
    scatters = [ax.scatter([], [], color=colors(i), marker='o', s=100, label=f'Hunter {i}') for i in range(huntersNum)]
    scatters.append(ax.scatter([], [], color='red', marker='*', s=120, label='Target'))  # Target point
    lines = [ax.plot([], [], color=colors(i))[0] for i in range(huntersNum)]
    lines.append(ax.plot([], [], color='red', linestyle='--')[0])  # Target path line

    # Add legend
    ax.legend()

    # Dynamic update for plotting
    ani = animation.FuncAnimation(fig, update, frames=len(target_positions),
                                  fargs=(hunter_positions, target_positions, scatters, lines),
                                  interval=gap, repeat=False)

    # Show animation
    plt.show()
