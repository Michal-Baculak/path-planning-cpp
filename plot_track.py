import matplotlib.pyplot as plt

def read_curves(filename):
    curves = []
    current_curve = []
    
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if line == '---':
                if current_curve:
                    curves.append(current_curve)
                    current_curve = []
            else:
                parts = line.split()
                if len(parts) == 2:
                    try:
                        x, y = map(float, parts)
                        current_curve.append((x, y))
                    except ValueError:
                        pass  # Ignore lines that can't be parsed
    
    if current_curve:  # Add the last curve if present
        curves.append(current_curve)
    
    return curves

def plot_curves(filename):
    curves = read_curves(filename)
    colors = ['b', 'r', 'g', 'c', 'm', 'y']  # Color choices
    
    plt.figure()
    for i, curve in enumerate(curves):
        if curve:
            x_vals, y_vals = zip(*curve)
            plt.plot(x_vals, y_vals, marker='o', linestyle='-', color=colors[i % len(colors)], label=f'Curve {i+1}')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Plot of Curves')
    plt.show()
    

# Example usage
filename = 'track.txt'  # Replace with your file name
plot_curves(filename)
