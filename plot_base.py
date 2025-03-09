import matplotlib.pyplot as plt

def read_lines(filename):
    lines = []
    
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            parts = line.split()
            if len(parts) == 4:
                try:
                    x1, y1, x2, y2 = map(float, parts)
                    lines.append((x1, y1, x2, y2))
                except ValueError:
                    pass  # Ignore lines that can't be parsed
    
    return lines

def plot_base(filename):
    lines = read_lines(filename)

    plt.figure()
    for line in lines:
        if line:
            (x1, y1, x2, y2) = line
            plt.plot((x1, x2), (y1, y2), linestyle='-', color = 'r')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Plot of track base')
    plt.show()

# Example usage
filename = 'base.txt'  # Replace with your file name
plot_base(filename)
