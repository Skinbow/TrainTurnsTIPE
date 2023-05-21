from matplotlib.pyplot import plot, show, grid, quiver

def readData(fname="data.csv"):
    with open(fname, "r") as file:
        X = [float(x) for x in file.readline().split(",")[:-1]]
        Y = [float(x) for x in file.readline().split(",")[:-1]]
    return (X, Y)
    


def draw():
    (X, Y) = readData()
    plot(X, Y)
    #quiver(X, Y, U, V)
    grid(True); show()

def main():
    draw()


if __name__ == "__main__":
    main()