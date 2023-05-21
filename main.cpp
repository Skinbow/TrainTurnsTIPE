#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <time.h>
#include <utility>
#include <fstream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Vector3d.hpp"

#define N_POINTS 10
#define POWER_OF_THREE 59049UL

std::ofstream file;

struct Trajectory {
    std::vector<Vector3d> points;
    Trajectory() {
        points = std::vector<Vector3d>();
    }
};

struct Train {
    double speed; // en m/s
    
    std::vector<std::vector<Vector3d>> positionDerivatives;

    Train() : speed(1) {}
    void calculateMovement(const Trajectory &trajectory) {
        size_t N = trajectory.points.size();
        positionDerivatives.clear();
        positionDerivatives.push_back(trajectory.points);

        std::vector<double> times = {0.0};

        double travelDistance = 0;
        for (int j = 0; j < N - 1; j++) {
            Vector3d dOM = positionDerivatives[0][j + 1] - positionDerivatives[0][j];
            travelDistance += dOM.norm();
            times.push_back(travelDistance / speed);
        }

        for (int j = 1; j < 4; j++) {
            positionDerivatives.push_back(std::vector<Vector3d>());
            for (int i = 0; i < N - j; i++) {
                positionDerivatives[j].push_back((positionDerivatives[j - 1][i + 1] - positionDerivatives[j - 1][i]) / (times[i + 1] - times[i]));
            }
        }
    }

    std::vector<std::vector<Vector3d>> getPositionDerivatives() { 
        return positionDerivatives; 
    }
};

void printPoints(const std::vector<Vector3d> &points) {
    for (Vector3d Point:points) {
        std::cout << "(" << Point.x << ", " << Point.y << ")" << std::endl;
    }
}

void printForPython(const std::vector<Vector3d> &points) {
    std::vector<double> X, Y;
    for (Vector3d Point:points) {
        X.push_back(Point.x);
        Y.push_back(Point.y);
    }
    std::cout << "[";
    for (double x : X) {
        std::cout << x << ",";
        file << x << ",";
    }
    std::cout << "]\n[";
    file << "\n";
    for (double y : Y) {
        std::cout << y << ",";
        file << y << ",";
    }
    std::cout << "]\n" << std::endl;
}

/////
// Utilisation du code de Gray

typedef unsigned int uint;

// This function converts an unsigned binary number to reflected binary Gray code.
void toTernaryGray(unsigned value, unsigned gray[N_POINTS])
{ 
	unsigned baseN[N_POINTS];	// Stores the ordinary base-N number, one digit per entry
	unsigned i;		// The loop variable
 
	// Put the normal baseN number into the baseN array. For base 10, 109 
	// would be stored as [9,0,1]
	for (i = 0; i < N_POINTS; i++) {
		baseN[i] = value % 3;
		value    = value / 3;
	}
 
	// Convert the normal baseN number into the Gray code equivalent. Note that
	// the loop starts at the most significant digit and goes down.
	unsigned shift = 0;
	while (i--) {
		// The Gray digit gets shifted down by the sum of the higher
		// digits.
		gray[i] = (baseN[i] + shift) % 3;
		shift = shift + 3 - gray[i];	// Subtract from base so shift is positive
	}
}

std::pair<size_t, int> ternaryGrayDifference(unsigned n)
{ 
	unsigned gray1[N_POINTS];
    unsigned gray2[N_POINTS];

    toTernaryGray(n, gray1);
    toTernaryGray(n + 1, gray2);

    unsigned change = -1; // Donne une valeur invalide
    for (int i = 0; i < N_POINTS; i++) {
        if (gray1[i] != gray2[i]) {
            change = i;
        }
    }
    int dir = ((int) gray2[change]) - ((int) gray1[change]);
    //std::cout << dir << std::endl;
    return std::make_pair(change, dir);
}
////

// On utilise une heuristique en choisissant un chemin proche
Trajectory getNearbyTrajectory(Trajectory traj, double step) {
    Trajectory newtraj;
    newtraj.points.push_back(traj.points[0]);
    newtraj.points.push_back(traj.points[1]);

    Vector3d dir = (traj.points[traj.points.size() - 2] - traj.points[1]);
    Vector3d normal = {dir.y, -dir.x, 0.0};
    normal = normal / normal.norm();

    // On laisse fixe les quatre points aux extremites, puisque les bouts sont lineaires
    for (int i = 2; i < traj.points.size() - 2; i++) {
        int r = rand();
        Vector3d add;
        if ((r & 2) == 0) {
            add = Vector3d::zero();
        }
        else {
            add = normal * ((double) ((r & 1) * 2 - 1));
        }
        newtraj.points.push_back(traj.points[i] + (add * step));
    }

    newtraj.points.push_back(traj.points[traj.points.size() - 2]);
    newtraj.points.push_back(traj.points[traj.points.size() - 1]);
    return newtraj;
}

// On utilise une heuristique en choisissant un chemin proche
void getNextTrajectory(Trajectory &traj, uint n, double step) {
    auto ch = ternaryGrayDifference(n);

    Vector3d dir = (traj.points[traj.points.size() - 2] - traj.points[1]);
    Vector3d normal = {dir.y, -dir.x, 0.0};
    normal = normal / normal.norm();
    //std::cout << traj.points.size() << std::endl;
    
    // On se deplace dans la direction orthogonale
    //if (ch.first == 9) std::cout << "Ten! " << n << " " << ch.second << " " << traj.points[2 +ch.first] << std::endl;
    traj.points[2 + ch.first] = traj.points[2 + ch.first] + normal * step * ch.second;
}

double valueFunction(std::vector<std::vector<Vector3d>> positionDerivatives) {
    // double jerkMax = 0;
    // for (auto jerk : positionDerivatives[3]) {
    //     double jn = jerk.norm();
    //     if (jn > jerkMax) {
    //         jerkMax = jn;
    //     }
    // }
    // return 1/jerkMax;

    double speedMax = 0;
    for (auto speed : positionDerivatives[2]) {
        double sn = speed.norm();
        if (sn > speedMax) {
            speedMax = sn;
        }
    }
    return -speedMax;
    // double travelDistance = 0;
    //     for (int j = 0; j < positionDerivatives[0].size() - 1; j++) {
    //         Vector3d dOM = positionDerivatives[0][j + 1] - positionDerivatives[0][j];
    //         travelDistance += dOM.norm();
    //     }
    //     return 1/travelDistance;
}

Trajectory getStraightTrajectory(Vector3d endpoints[4], int N) {
    Trajectory traj;
    traj.points.push_back(endpoints[0]);
    traj.points.push_back(endpoints[1]);
    Vector3d dir = (endpoints[2] - endpoints[1]) / (N + 1);
    for (int i = 1; i < N + 1; i++) {
        traj.points.push_back(endpoints[1] + (dir * i));
    }
    traj.points.push_back(endpoints[2]);
    traj.points.push_back(endpoints[3]);
    std::cout << traj.points.size();
    return traj;
}

// On randomise en suivant la direction generale entre les deux bouts
Trajectory getRandomTrajectory(Vector3d endpoints[4], int N) {
    Trajectory traj;
    traj.points.push_back(endpoints[0]);
    traj.points.push_back(endpoints[1]);
    Vector3d dir = (endpoints[2] - endpoints[1]) / N;
    Vector3d normal = {dir.y, -dir.x, 0.0};
    normal = normal / (normal.norm());
    double range = dir.norm() * N / 1000;
    for (int i = 1; i < N; i++) {
        double t = 2 * range  * (random() / (double) RAND_MAX) - range;
        traj.points.push_back(endpoints[1] + (dir * i) + normal * t);
    }
    traj.points.push_back(endpoints[2]);
    traj.points.push_back(endpoints[3]);
    return traj;
}

std::pair<Trajectory, double> remonterGradient(Trajectory &traj_0, double tolerance) {
    Train train;
    int maxTries = 1000;
    double step = tolerance * 1000;
    Trajectory optimalTraj;
    double optimalValue;

    train.calculateMovement(traj_0);
    optimalValue = valueFunction(train.getPositionDerivatives());
    optimalTraj = traj_0;

    Trajectory tempTraj;
    double tempValue = 0;
    std::vector<std::vector<Vector3d>> positionDerivatives;
    while (step > tolerance) {
        int noRiseTries = 0;
        while (noRiseTries < maxTries) {
            tempTraj = getNearbyTrajectory(optimalTraj, step);
            train.calculateMovement(tempTraj);
            tempValue = valueFunction(train.getPositionDerivatives());
            // std::cout << -tempValue << ", " << step << std::endl;
            if (tempValue > optimalValue) {
                optimalTraj = tempTraj;
                optimalValue = tempValue;
                noRiseTries = 0;
            }
            else {
                noRiseTries += 1;
            }
        }
        step /= 2;
    }
    return std::make_pair(optimalTraj, optimalValue);
}

void slide(Trajectory &traj, double step) {
    Vector3d dir = (traj.points[traj.points.size() - 2] - traj.points[1]);
    Vector3d normal = {dir.y, -dir.x, 0.0};
    normal = normal / normal.norm();

    for (int i = 2; i < traj.points.size() - 2; i++) {
        traj.points[i] = traj.points[i] + normal * (-1) * step;
    }
}

std::pair<Trajectory, double> remonterGradientExact(Trajectory &traj_0, double tolerance) {
    Train train;
    double step = 0.1;
    Trajectory optimalTraj;
    double optimalValue;

    train.calculateMovement(traj_0);
    optimalValue = valueFunction(train.getPositionDerivatives());
    optimalTraj = traj_0;

    Trajectory tempTraj;
    double tempValue = 0;
    
    std::vector<std::vector<Vector3d>> positionDerivatives;
    while (step > tolerance) {
        bool noImprovement = false;
        while (!noImprovement) {
            tempTraj = optimalTraj;
            slide(tempTraj, step);
            noImprovement = true;
            for (int i = 0; i < POWER_OF_THREE; i++) {
                //std::cout << tempTraj.points[tempTraj.points.size() - 3] << std::endl;
                getNextTrajectory(tempTraj, i, step);
                train.calculateMovement(tempTraj);
                tempValue = valueFunction(train.getPositionDerivatives());
                // std::cout << -tempValue << ", " << step << std::endl;
                
                if (tempValue > optimalValue + 0.00001) {
                    std::cout << tempValue <<  " " << optimalValue << std::endl;
                    optimalTraj = tempTraj;
                    optimalValue = tempValue;
                    noImprovement = false;
                    std::cout << step << std::endl;
                }
            }
            std::cout << noImprovement << std::endl;
        }
        step /= 2;
    }
    return std::make_pair(optimalTraj, optimalValue);
}

void findBestTrajectoryExact(int nTries) {
    Trajectory traj;
    Trajectory optimalTraj;
    double optimalValue = -INFINITY;
    Vector3d endpoints[4] = {{-0.05, 0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {1.05, 1.0, 0.0}};
    
    traj = getStraightTrajectory(endpoints, N_POINTS);
    auto ret = remonterGradientExact(traj, 0.006);
    
    optimalTraj = ret.first;
    optimalValue = ret.second;

    std::cout << optimalValue << std::endl;
    Train train;
    train.calculateMovement(optimalTraj);
    //printForPython(train.getPositionDerivatives()[2]);
    printPoints(optimalTraj.points);
    printForPython(optimalTraj.points);
    
    return;
}

// void findBestTrajectory(int nTries) {
//     Trajectory traj;
//     Trajectory optimalTraj;
//     double optimalValue = -INFINITY;
//     Vector3d endpoints[4] = {{-0.05, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {1.05, 1.0, 0.0}};
//     for (int i = 0; i < nTries; i++) {
//         std::cout << "Try number " << (i+1) << std::endl;
//         traj = getStraightTrajectory(endpoints, 100);
//         auto ret = remonterGradient(traj, 0.0000001);
//         if (optimalValue < ret.second) {
//             optimalTraj = ret.first;
//             optimalValue = ret.second;
//         }
//     }
//     std::cout << optimalValue << std::endl;
//     Train train;
//     train.calculateMovement(optimalTraj);
//     printForPython(train.getPositionDerivatives()[2]);
//     printPoints(optimalTraj.points);
//     printForPython(optimalTraj.points);
//     return;
// }

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}  

void processInput(GLFWwindow *window) {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

int pow3(unsigned n) {
    unsigned p = 1;
    for (int i = 0; i < n; i++) {
        p *= 3;
    }
    return p;
}

int main() {
    srand(time(NULL));
    file.open("data.csv");

    findBestTrajectoryExact(1);

    file.close();

    // int cstd::ofstream filehecked[59050];
    // for (int i = 0; i <= 59049; i++) {
    //     checked[i] = 0;
    // }
    // unsigned x = 0;
    // for (int i = 0; i <= 59049; i++){
    //     auto p = ternaryGrayDifference(i);
    //     x += p.second * pow3(p.first);
    //     checked[x] += 1;
    //     //std::cout << p.first << " " << p.second << std::endl;
    // }

    // for (int i = 0; i < 59050; i++) {
    //     std::cout << i << " " << checked[i] << std::endl;
    // }
    // Trajectory traj;
    // for (int i = 0; i < 100; i++)
    // {
    //     traj.points.push_back({i/100.0, 0.0, 0.0});
    // }
    
    // Vector3d endpoints[4] = {{-0.05, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.05, 0.0, 0.0}};
    // Trajectory traj = getRandomTrajectory(endpoints, 10);
    // Trajectory newTraj = getNearbyTrajectory(traj, 0.005);
    
    // Train train;
    // train.calculateMovement(traj);
    // auto pos = train.getPositionDerivatives();
    // printPoints(pos[3]);
    // std::cout << pos[0].size() << " " << pos[1].size() << " " << pos[2].size() << " " << pos[3].size() << std::endl;
    // printTrajectory(traj);
    // printTrajectory(newTraj);

    // glfwInit();
    // glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    // glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // GLFWwindow* window = glfwCreateWindow(800, 600, "LearnOpenGL", NULL, NULL);
    // if (window == NULL)
    // {
    //     std::cout << "Failed to create GLFW window" << std::endl;
    //     glfwTerminate();
    //     return -1;
    // }
    // glfwMakeContextCurrent(window);

    // if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    // {
    //     std::cout << "Failed to initialize GLAD" << std::endl;
    //     return -1;
    // }    

    // glViewport(0, 0, 800, 600);

    // glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

    // while (!glfwWindowShouldClose(window))
    // {
    //     processInput(window);

    //     glClear(GL_COLOR_BUFFER_BIT);

    //     glfwSwapBuffers(window);
    //     glfwPollEvents();
    // }

    // glfwTerminate();
    return 0;
}