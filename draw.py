from matplotlib.pyplot import plot, show, grid, quiver

def draw():
    X = [-0.05,0,0.1572,0.261368,0.33018,0.354798,0.34406,0.412872,0.503781,0.59911,0.690019,0.816283,1,1.05,]
    Y = [0,0,0.0246178,0.102269,0.215275,0.372475,0.565031,0.678037,0.768946,0.855436,0.946345,1.0019,1,1,]
    plot(X, Y)
    #quiver(X, Y, U, V)
    grid(True); show()

def main():
    draw()


if __name__ == "__main__":
    main()