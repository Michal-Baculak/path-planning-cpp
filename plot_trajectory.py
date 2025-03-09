import matplotlib.pyplot as plt

def read_points(filename):
    Points = []
    
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            parts = line.split()
            if len(parts) == 2:
                try:
                    x, y = map(float, parts)
                    Points.append((x, y))
                except ValueError:
                    pass  # Ignore lines that can't be parsed
    return Points

def plot_trajectory(filename):
    points = read_points(filename)
    plt.figure()
    x_vals = []
    y_vals= []
    for point in points:
        if point:
            x_vals.append(point[0])
            y_vals.append(point[1])
    plt.plot(x_vals, y_vals, marker='o', linestyle='-', color='r')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Trajectory')
    plt.gca().set_aspect("equal")
    plt.show()
    

# Example usage
filename = 'trajectory.txt'  # Replace with your file name
plot_trajectory(filename)
