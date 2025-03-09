import matplotlib.pyplot as plt

def plot_all(innConsX, innConsY, outConsX, outConsY, base, ptsX, ptsY):
    plt.figure()
    plt.plot(innConsX, innConsY, marker="o", linestyle="-", color="y")
    plt.plot(outConsX, outConsY, marker="o", linestyle="-", color="b")
    plt.plot(ptsX, ptsY, marker="x", linestyle="-", color="k")
    for line in base:
        if line:
            (x1, y1, x2, y2) = line
            plt.plot((x1, x2), (y1, y2), linestyle='-', color = 'r')
    plt.gca().set_aspect("equal")
    plt.show()

# plot_all([1,2,3],[1,2,3], [4,5,6], [4,5,6], [(1,2,3,4),(5,6,7,8),(9,10,11,12)], [10,11,12,13],[10,11,12,13])