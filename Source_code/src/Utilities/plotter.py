import matplotlib.pyplot as plt
import sys
import pandas as pd

SIDE = 200


def scatterplot(_df, x_dim, y_dim):
    x = _df[x_dim]
    y = _df[y_dim]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.scatter(x, y, c='green', alpha=0.5)

    # adds a title and axes labels
    ax.set_title('Board Visualization')
    ax.set_xlabel('X coordinate')
    ax.set_ylabel('Y coordinate')

    # removing top and right borders
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    # adds major grid lines
    ax.grid(color='black', linestyle='-', linewidth=0.25, alpha=0.5)
    ax.set_xlim(xmin=0, xmax=SIDE)
    ax.set_ylim(ymin=0, ymax=SIDE)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


def read(read_data, x, y, time):
    line_idx = 0
    while line_idx < len(read_data):
        line = read_data[line_idx]
        if line[0] == '-':
            line_idx += 1
            continue
        if line[0:3] == 'EOF':
            line_idx += 1
            break
        data = line.split()
        if len(data) == 1:
            x.append(int(data[0]))
        elif len(data) == 2:
            if time:
                y.append(float(data[1])/1000)
            else:
                y.append(float(data[0]))
        line_idx += 1


def main():
    if len(sys.argv) != 2:
        print("Incorrect usage of command line arguments!\n"
              "Usage:\n"
              "(1) Path of csv file to visualize")
        quit()
    # ---- Print Grid ----
    df = pd.read_csv(sys.argv[1])
    df.columns = ['x', 'y']
    scatterplot(df, 'x', 'y')


if __name__ == "__main__":
    main()
