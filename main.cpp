#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <random>
#include <time.h>
#include <utility>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Vector3d.hpp"

#define N_POINTS 11
#define POWER_OF_THREE 177147UL//59049UL //

std::ofstream file;

struct Trajectory {
    std::vector<Vector3d> points;
    Trajectory() {
        points = std::vector<Vector3d>();
    }
};

struct Train {
    double speed; // en m/s
    std::vector<double> times;
    
    std::vector<std::vector<Vector3d>> positionDerivatives;

    Train() : speed(1) {}
    void calculateMovement(const Trajectory &trajectory) {
        positionDerivatives.clear();
        positionDerivatives.push_back(trajectory.points);

        size_t N = trajectory.points.size();

        times.clear();
        times.push_back(0.0);

        std::vector<double> doubleTimes;
        doubleTimes.push_back(0.0);

        double travelDistance = 0;
        for (unsigned int j = 0; j < N - 1; j++) {
            Vector3d dOM = positionDerivatives[0][j + 1] - positionDerivatives[0][j];
            travelDistance += dOM.norm();
            doubleTimes.push_back((travelDistance - dOM.norm()/2) / speed);
            doubleTimes.push_back(travelDistance / speed);
            times.push_back(travelDistance / speed);
        }

        for (unsigned j = 1; j < 4; j++) {
            positionDerivatives.push_back(std::vector<Vector3d>());
            for (unsigned i = 0; i < N - j; i++) {
                positionDerivatives[j].push_back((positionDerivatives[j - 1][i + 1] - positionDerivatives[j - 1][i]) / (doubleTimes[j - 1 + 2 * (i + 1)] - doubleTimes[j - 1 + 2 * i]));
            }
        }
    }

    std::vector<std::vector<Vector3d>> getPositionDerivatives() { 
        return positionDerivatives; 
    }
};

void printPoints(const std::vector<Vector3d> &points) {
    std::vector<double> X, Y;
    for (Vector3d Point:points) {
        std::cout << "(" << Point.x << ", " << Point.y << ")" << std::endl;
        X.push_back(Point.x);
        Y.push_back(Point.y);
    }
    for (auto x : X) {
        std::cout << /*std::setprecision(4) <<*/  x << " ";
    }
    std::cout << "\n";
    for (auto y : Y) {
        std::cout << y << " ";
    }
    std::cout.flush();
}

void writeToFile(double x) {
    file << x << ", ";
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

// This function converts an unsigned binary number to reflected binary Gray code.
void toTernaryGray(unsigned value, unsigned gray[N_POINTS])
{ 
    unsigned baseN[N_POINTS];    // Stores the ordinary base-N number, one digit per entry
    unsigned i;    // The loop variable
 
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
        shift = shift + 3 - gray[i];    // Subtract from base so shift is positive
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
    
    return std::make_pair(change, dir);
}

// On utilise une heuristique en choisissant un chemin proche
void getNextTrajectory(Trajectory &traj, uint n, double step) {
    auto ch = ternaryGrayDifference(n);

    Vector3d dir = (traj.points[traj.points.size() - 3] - traj.points[2]);
    Vector3d normal = {dir.y, -dir.x, 0.0};
    normal = normal / normal.norm();
    traj.points[3 + ch.first] = traj.points[3 + ch.first] + normal * step * ch.second;
}

double valueFunction(std::vector<std::vector<Vector3d>> positionDerivatives, const Train& train) {
    auto& times = train.times;

    std::vector<double> curvatureDer;
    for (unsigned int i = 0; i < positionDerivatives[2].size() - 1; i++) {
        curvatureDer.push_back((positionDerivatives[2][i + 1].norm() - positionDerivatives[2][i].norm())/(times[i + 2] - times[i + 1]));
    }
    double maxCurDer = 0;
    for (unsigned int i = 0; i < curvatureDer.size(); i++) {
        double sn = curvatureDer[i] * ((times[i + 2] - times[i + 1]));
        if (abs(sn) > maxCurDer) {
            maxCurDer = abs(sn);
        }
    }

    //double maxAcc = 0;
    double sumAcc = 0;
    for (unsigned int i = 0; i < positionDerivatives[2].size(); i++) {
        double sn = positionDerivatives[2][i].norm() * ((times[i + 2] - times[i]) / 2);
        sumAcc += sn;
        //if (sn > maxAcc) {
        //    maxAcc = sn;
        //}
    }

    double sumJerk = 0;
    double maxJerk = 0;
    for (unsigned int i = 0; i < positionDerivatives[3].size(); i++) {
        double sn = positionDerivatives[3][i].norm() * (times[i + 2] - times[i + 1]);
        sumJerk += sn;
        if (sn > maxJerk) {
            maxJerk = sn;
        }
    }

    return /*3 * maxAcc + sumAcc + 2 * maxJerk + sumJerk*/ sumJerk + sumAcc + maxJerk + 10*maxCurDer;
}

Trajectory getStraightTrajectory(Vector3d endpoints[6], int N) {
    Trajectory traj;
    traj.points.push_back(endpoints[0]);
    traj.points.push_back(endpoints[1]);
    traj.points.push_back(endpoints[2]);
    Vector3d dir = (endpoints[3] - endpoints[2]) / (N + 1);
    for (int i = 1; i < N + 1; i++) {
        traj.points.push_back(endpoints[2] + (dir * i));
    }
    traj.points.push_back(endpoints[3]);
    traj.points.push_back(endpoints[4]);
    traj.points.push_back(endpoints[5]);
    //std::cout << traj.points.size();
    return traj;
}

// Decale la trajectoire de depart d'une distance step sur le côté pour pouvoir utiliser le codage ternaire de gray
void slide(Trajectory &traj, double step) {
    Vector3d dir = (traj.points[traj.points.size() - 3] - traj.points[2]);
    Vector3d normal = {dir.y, -dir.x, 0.0};
    normal = normal / normal.norm();

    for (unsigned int i = 3; i < traj.points.size() - 3; i++) {
        traj.points[i] = traj.points[i] + normal * (-1) * step;
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}  

void processInput(GLFWwindow *window) {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

const char* vertexShaderSource = "#version 460 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "void main()"
        "{"
        "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);"
        "}";

const char* fragmentShaderSource = "#version 460 core\n"
        "out vec4 FragColor;\n"
        "void main()"
        "{"
        "FragColor = vec4(0.9f, 0.0f, 0.0f, 1.0f);"
        "}";

unsigned int genShaderProgram() {
    
    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);


    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    int success;
    char infoLog[512];

    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
    glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    unsigned int shaderProgram;
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);


    
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if(!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cout << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProgram;
}

void draw(unsigned int& VAO, unsigned int& VBO, unsigned int& shaderProgram, unsigned int n, unsigned int m) {
    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINE_STRIP, 0, n);
    //glDrawArrays(GL_LINES, n, m);
}

void setup(unsigned int& VAO, unsigned int& VBO, unsigned int& shaderProgram) {
    shaderProgram = genShaderProgram();

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void writeTrajectoryBuffer(unsigned int& VAO, unsigned int& VBO, Trajectory& optimalTraj, std::vector<Vector3d> jerkVectors) {
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    auto n = optimalTraj.points.size();

    float traj[3 * (n + jerkVectors.size())];
    for (unsigned int i = 0; i < optimalTraj.points.size(); i++) {
        traj[3*i] = optimalTraj.points[i].x;
        traj[3*i + 1] = optimalTraj.points[i].y;
        traj[3*i + 2] = optimalTraj.points[i].z;
    }

    for (unsigned int i = 0; i < jerkVectors.size(); i++) {
        traj[3 * n + 3*i] = jerkVectors[i].x;
        traj[3 * n + 3*i + 1] = jerkVectors[i].y;
        traj[3 * n + 3*i + 2] = jerkVectors[i].z;
    }

    glBufferData(GL_ARRAY_BUFFER, sizeof(traj), traj, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

int main() {
    srand(time(NULL));
    file.open("data.csv");

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1000, 1000, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }    

    glViewport(0, 0, 1000, 1000);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    unsigned int VAO, VBO, shaderProgram;
    setup(VAO, VBO, shaderProgram);

////////// Train logic
    Trajectory traj;
    Trajectory optimalTraj;
    double optimalValue = -INFINITY;
    double s = 0.9;
    Vector3d endpoints[6] = {{-1.0*s, -1.05*s, 0.0*s}, {-1.0*s, -1.0*s, 0.0*s}, {-1.0*s, -0.95*s, 0.0*s}, {1.0*s, 0.95*s, 0.0*s}, {1.0*s, 1.0*s, 0.0*s}, {1.0*s, 1.05*s, 0.0*s}};
    traj = getStraightTrajectory(endpoints, N_POINTS);


    Train train;
    double step = 0.1;

    train.calculateMovement(traj);
    optimalValue = valueFunction(train.getPositionDerivatives(), train);
    optimalTraj = traj;

    Trajectory tempTraj;
    double tempValue = 0;

    std::vector<std::vector<Vector3d>> positionDerivatives;
    double tolerance = 0.00000001;
    bool valueImproved = true;
///////////
    auto start = std::chrono::steady_clock::now();
    std::vector<Vector3d> jerkVectors;
    while (!glfwWindowShouldClose(window))
    {
        processInput(window);

        glClear(GL_COLOR_BUFFER_BIT);

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> delay = now - start;
        if (step > tolerance) {
            if (valueImproved) {
                if (delay.count() > 0) {
                    tempTraj = optimalTraj;
                    slide(tempTraj, step);
                    valueImproved = false;
                    double values[POWER_OF_THREE];
                    for (unsigned int i = 0; i < POWER_OF_THREE; i++) {
                        //std::cout << tempTraj.points[tempTraj.points.size() - 3] << std::endl;
                        getNextTrajectory(tempTraj, i, step);
                        train.calculateMovement(tempTraj);
                        tempValue = valueFunction(train.getPositionDerivatives(), train);
                        values[i] = tempValue;
                        // std::cout << -tempValue << ", " << step << std::endl;
                        
                        if (tempValue < optimalValue - 0.000000001 /*&& (rand()%20) == 0*/) {
                            optimalTraj = tempTraj;
                            optimalValue = tempValue;
                            valueImproved = true;
                            std::cout << "Better value: " << optimalValue << std::endl;
                            std::cout << "Step: " << step << std::endl;
                        }
                    }
                    writeToFile(optimalValue);
                    start = std::chrono::steady_clock::now();
                }
            }
            else {
                std::cout << "Best value at this step: " << optimalValue << std::endl;
                std::cout << "Step: " << step << std::endl;
                valueImproved = true;
                step /= 2;
            }
            jerkVectors.clear();
            train.calculateMovement(optimalTraj);
            auto pos = train.getPositionDerivatives();
            double max = 0;
            for (unsigned int i = 0; i < pos[2].size(); i++) {
                double norm = pos[2][i].norm();
                if (norm > max) {
                    max = norm;
                }
            }
            for (int i = 0; i < pos[2].size(); i++) {
                auto mid = pos[0][i + 1];// + (pos[0][i + 2] - pos[0][i + 1]) / 2;
                jerkVectors.push_back(mid);
                jerkVectors.push_back(mid + pos[2][i] / (5*max));
            }
            writeTrajectoryBuffer(VAO, VBO, optimalTraj, jerkVectors);
        }
        draw(VAO, VBO, shaderProgram, optimalTraj.points.size(), jerkVectors.size());
////////////////
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    std::cout << "The best value: " << optimalValue << std::endl;
    train.calculateMovement(optimalTraj);
    printPoints(train.getPositionDerivatives()[0]);

    std::cout << std::endl;
    std::cout << valueFunction(train.getPositionDerivatives(), train) << std::endl;

    file.close();
    glfwTerminate();
    return 0;
}