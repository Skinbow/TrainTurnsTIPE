#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <random>
#include <time.h>
#include <utility>
#include <fstream>
#include <chrono>
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
        positionDerivatives.clear();
        // positionDerivatives.push_back(std::vector<Vector3d>());
        // positionDerivatives[0].push_back(trajectory.points[0]);
        // for (int i = 0; i < trajectory.points.size() - 1; i++) {
        //     positionDerivatives[0].push_back(trajectory.points[i+1] + trajectory.points[i]);
        //     positionDerivatives[0].push_back(trajectory.points[i + 1]);
        // }
        positionDerivatives.push_back(trajectory.points);

        size_t N = trajectory.points.size();

        std::vector<double> times = {0.0};

        double travelDistance = 0;
        for (unsigned int j = 0; j < N - 1; j++) {
            Vector3d dOM = positionDerivatives[0][j + 1] - positionDerivatives[0][j];
            travelDistance += dOM.norm();
            times.push_back(travelDistance / speed);
        }

        for (unsigned j = 1; j < 4; j++) {
            positionDerivatives.push_back(std::vector<Vector3d>());
            for (unsigned i = 0; i < N - j; i++) {
                positionDerivatives[j].push_back((positionDerivatives[j - 1][i + 1] - positionDerivatives[j - 1][i]) / (times[i + 1] - times[i]));
            }
            //std::cout << positionDerivatives[j - 1].size() << " " << N - j << std::endl;
            // for (int i = 1; i < N - j; i++) {
            //     positionDerivatives[j].push_back((positionDerivatives[j - 1][i + 1] - positionDerivatives[j - 1][i - 1]) / (times[i + 1] - times[i - 1]));
            // }
        }

        
        positionDerivatives[2].clear();
        for (unsigned i = 1; i < N - 1; i++) {
            positionDerivatives[2].push_back((positionDerivatives[0][i + 1] + positionDerivatives[0][i - 1] - positionDerivatives[0][i] * 2) / ((times[i + 1] - times[i - 1]) * (times[i + 1] - times[i - 1])));
        }
        // for (unsigned i = 0; i < N - 3; i++) {
        //     std::cout << positionDerivatives[2][i] << " ";
        // }
        // std::cout << std::endl;
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
    // for (int i = 0; i < N_POINTS; i++) {
    //     std::cout << gray1[i];
    // }
    // std::cout << std::endl;
    
    return std::make_pair(change, dir);
}

// On utilise une heuristique en choisissant un chemin proche
void getNextTrajectory(Trajectory &traj, uint n, double step) {
    auto ch = ternaryGrayDifference(n);

    Vector3d dir = (traj.points[traj.points.size() - 3] - traj.points[2]);
    Vector3d normal = {dir.y, -dir.x, 0.0};
    normal = normal / normal.norm();
    //std::cout << traj.points.size() << std::endl;
    
    // On se deplace dans la direction orthogonale
    //if (ch.first == 9) std::cout << "Ten! " << n << " " << ch.second << " " << traj.points[2 +ch.first] << std::endl;
    traj.points[3 + ch.first] = traj.points[3 + ch.first] + normal * step * ch.second;
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

    // double speedMax = 0;
    // for (auto speed : positionDerivatives[2]) {
    //     double sn = speed.norm();
    //     if (sn > speedMax) {
    //         speedMax = sn;
    //     }
    // }

    double max = 0;
    double sum = 0;
    for (unsigned int i = 0; i < positionDerivatives[2].size(); i++) {
        auto direction = positionDerivatives[0][i + 1] - positionDerivatives[0][i];
        auto normal = direction.cross(Vector3d(0,0,1));
        normal = normal * 1/normal.norm();
        double sn = positionDerivatives[2][i].dot(normal);
        sum += sn*sn;
        if (sn*sn > max) {
            max = sn*sn;
        }
    }
    sum = sum / positionDerivatives[2].size();
    return max + sum;

    // double max = 0;
    // for (auto speed : positionDerivatives[2]) {
    //     double sn = speed.norm();
    //     if (sn*sn > max) {
    //         max = sn*sn;
    //     }
    // }

    return max;
    // double travelDistance = 0;
    //     for (int j = 0; j < positionDerivatives[0].size() - 1; j++) {
    //         Vector3d dOM = positionDerivatives[0][j + 1] - positionDerivatives[0][j];
    //         travelDistance += dOM.norm();
    //     }
    //     return 1/travelDistance;
}

Trajectory getStraightTrajectory(Vector3d endpoints[6], int N) {
    Trajectory traj;
    traj.points.push_back(endpoints[0]);
    traj.points.push_back(endpoints[1]);
    traj.points.push_back(endpoints[2]);
    Vector3d dir = (endpoints[3] - endpoints[2]) / (N + 1);
    for (int i = 1; i < N + 1; i++) {
        traj.points.push_back(endpoints[1] + (dir * i));
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

// std::pair<Trajectory, double> remonterGradientExact(Trajectory &traj_0, double tolerance) {
//     Train train;
//     double step = 0.1;
//     Trajectory optimalTraj;
//     double optimalValue;

//     train.calculateMovement(traj_0);
//     optimalValue = valueFunction(train.getPositionDerivatives());
//     optimalTraj = traj_0;

//     Trajectory tempTraj;
//     double tempValue = 0;
    
//     std::vector<std::vector<Vector3d>> positionDerivatives;
//     while (step > tolerance) {
//         bool valueImproved = true;
//         while (valueImproved) {
//             tempTraj = optimalTraj;
//             slide(tempTraj, step);
//             valueImproved = false;
//             for (int i = 0; i < POWER_OF_THREE; i++) {
//                 //std::cout << tempTraj.points[tempTraj.points.size() - 3] << std::endl;
//                 getNextTrajectory(tempTraj, i, step);
//                 train.calculateMovement(tempTraj);
//                 tempValue = valueFunction(train.getPositionDerivatives());
//                 // std::cout << -tempValue << ", " << step << std::endl;
                
//                 if (tempValue < optimalValue - 0.00001) {
//                     std::cout << tempValue <<  " " << optimalValue << std::endl;
//                     optimalTraj = tempTraj;
//                     optimalValue = tempValue;
//                     valueImproved = true;
//                     std::cout << step << std::endl;
//                 }
//             }
//             std::cout << valueImproved << std::endl;
//         }
//         step /= 2;
//     }
//     return std::make_pair(optimalTraj, optimalValue);
// }

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}  

void processInput(GLFWwindow *window) {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

const float vertices[] = {
    -0.5f, -0.5f, 0.0f,
     0.5f, -0.5f, 0.0f,
     0.0f,  0.5f, -1.0f
};

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
        "FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);"
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

void draw(unsigned int& VAO, unsigned int& VBO, unsigned int& shaderProgram, unsigned int n) {
    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINE_STRIP, 0, n);
}

void setup(unsigned int& VAO, unsigned int& VBO, unsigned int& shaderProgram) {
    shaderProgram = genShaderProgram();

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void writeTrajectoryBuffer(unsigned int& VAO, unsigned int& VBO, Trajectory& optimalTraj) {
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    float traj[optimalTraj.points.size() * sizeof(float)];
    for (unsigned int i = 0; i < optimalTraj.points.size(); i++) {
        traj[3*i] = optimalTraj.points[i].x;
        traj[3*i + 1] = optimalTraj.points[i].y;
        traj[3*i + 2] = optimalTraj.points[i].z;
    }

    glBufferData(GL_ARRAY_BUFFER, sizeof(traj), traj, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

int main() {
    srand(time(NULL));
    file.open("data.csv");
    file.close();

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

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

    unsigned int VAO, VBO, shaderProgram;
    setup(VAO, VBO, shaderProgram);

////////// Train logic
    Trajectory traj;
    Trajectory optimalTraj;
    double optimalValue = -INFINITY;
    double s = 0.9;
    Vector3d endpoints[6] = {{-1.0*s, -1.05*s, 0.0*s}, {-1.0*s, -1.0*s, 0.0*s}, {-1.0*s, -0.95*s, 0.0*s}, {1.0*s, 1.0*s, 0.0*s}, {1.0*s, 1.05*s, 0.0*s}, {1.0*s, 1.1*s, 0.0*s}};
    traj = getStraightTrajectory(endpoints, N_POINTS);


    Train train;
    double step = 0.01;

    train.calculateMovement(traj);
    optimalValue = valueFunction(train.getPositionDerivatives());
    optimalTraj = traj;

    Trajectory tempTraj;
    double tempValue = 0;

    std::vector<std::vector<Vector3d>> positionDerivatives;
    double tolerance = 0.000000000001;
    bool valueImproved = true;
///////////
    auto start = std::chrono::steady_clock::now();
    while (!glfwWindowShouldClose(window))
    {
        processInput(window);

        glClear(GL_COLOR_BUFFER_BIT);

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> delay = now - start;
        if (step > tolerance) {
            if (valueImproved) {
                if (delay.count() > 0.125) {
                    tempTraj = optimalTraj;
                    slide(tempTraj, step);
                    valueImproved = false;
                    double values[POWER_OF_THREE];
                    for (unsigned int i = 0; i < POWER_OF_THREE; i++) {
                        //std::cout << tempTraj.points[tempTraj.points.size() - 3] << std::endl;
                        getNextTrajectory(tempTraj, i, step);
                        train.calculateMovement(tempTraj);
                        tempValue = valueFunction(train.getPositionDerivatives());
                        values[i] = tempValue;
                        // std::cout << -tempValue << ", " << step << std::endl;
                        
                        if (tempValue < optimalValue - 0.0001 && (rand()%2) == 0) {
                            optimalTraj = tempTraj;
                            optimalValue = tempValue;
                            valueImproved = true;
                            std::cout << "Better value: " << optimalValue << std::endl;
                            std::cout << "Step: " << step << std::endl;
                        }
                    }
                    // std::sort(values, values + POWER_OF_THREE, std::greater<double>());
                    // for (unsigned int i = 0; i < POWER_OF_THREE - 1; i++) {
                    //     std::cout << values[i] << "\t\t";
                    // }
                    // std::cout << values[POWER_OF_THREE - 1] << "\n" << std::endl;
                    //std::cout << valueImproved << std::endl;
                    start = std::chrono::steady_clock::now();
                }
            }
            else {
                std::cout << "Best value at this step: " << optimalValue << std::endl;
                std::cout << "Step: " << step << std::endl;
                valueImproved = true;
                step /= 2;
            }
            writeTrajectoryBuffer(VAO, VBO, optimalTraj);
        }
        draw(VAO, VBO, shaderProgram, optimalTraj.points.size());
////////////////
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    std::cout << "The best value: " << optimalValue << std::endl;
    train.calculateMovement(optimalTraj);
    //printForPython(train.getPositionDerivatives()[2]);
    //printPoints(optimalTraj.points);
    printPoints(train.getPositionDerivatives()[2]);
    //printForPython(optimalTraj.points);

    glfwTerminate();
    return 0;
}