#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <ctime>

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// Enum für verschiedene LiPo-Batterietypen
enum class BatteryType {
    LIPO_1S,  // 3.7V nominal
    LIPO_2S,  // 7.4V nominal
    LIPO_3S,  // 11.1V nominal
    LIPO_4S,  // 14.8V nominal
    LIPO_5S,  // 18.5V nominal
    LIPO_6S   // 22.2V nominal
};

// In ControlSettings, die Rotationssensitivität erhöhen:
struct ControlSettings {
    float rotationSensitivity = 1.2f; // Von 0.9 auf 1.2 erhöht
    float maxRotationRate = glm::radians(1200.0f); // Von 1000 auf 1200 erhöht
    float throttleIncrement = 0.4f;
    float throttlePosition = 0.0f;
    float throttleResponseRate = 3.0f;
    bool throttleReturnToZero = false;
};

ControlSettings controls;

// Diese Parameter sind für eine moderne Freestyle-Drohne realistischer
struct DroneConfig {
    float mass = 0.42f;                // Leichtere Masse für bessere Leistung
    float frameSize = 0.23f;
    float batteryCapacity = 1500.0f;
    float motorKv = 2300.0f;           // Hochwertige Race-Motoren
    float propSize = 5.0f;
    float motorResponseRate = 16.0f;
    float propEfficiency = 0.95f;      // Von 0.92 auf 0.95 erhöht
    float minBatteryVoltageForFlight = 3.3f * 4.0f;  // Realistisch für 4S

    float motorAcceleration = 50.0f;   // Schnellere Reaktion
    float motorDeceleration = 10.0f;
    float motorInertia = 0.000018f;
    float motorStartupThreshold = 0.05f;

    float thrustCoefficient = 0.52f;   // Diesen Wert deutlich erhöhen für realistischen Schub

    BatteryType batteryType = BatteryType::LIPO_4S;  // 4S ist Standard für Freestyle
};

DroneConfig droneConfig;

// Add this forward declaration at the top of the file, after structure definitions
void initializeDrone();

// Realer Gravitationswert
const float gravityForce = 9.81f;
const float rotationalDrag = 0.3f;
const float maxThrottle = 1.0f;

const int NUM_MOTORS = 4;
struct Motor {
    float targetThrottle = 0.0f;
    float currentThrottle = 0.0f;
    float currentRPM = 0.0f;
    float maxRPM = 0.0f;
    glm::vec3 position = glm::vec3(0.0f);
    float thrust = 0.0f;
    float currentDraw = 0.0f;

    float targetRPM = 0.0f;
    float torque = 0.0f;
    float rotationDir = 1.0f;
    float angularVelocity = 0.0f;
    bool isSpinning = false;
};

Motor motors[NUM_MOTORS];

// Reduziere die Luftwiderstandskoeffizienten für höhere Geschwindigkeit
struct Aerodynamics {
    // Reduzierte Widerstandskoeffizienten für höhere Geschwindigkeit
    glm::vec3 dragCoefficients = glm::vec3(0.2f, 0.3f, 0.2f); // Von (0.4, 0.6, 0.4)
    float groundEffectHeight = 0.5f;
    float groundEffectStrength = 1.3f;
    float propWashEffect = 0.12f;
    float airDensity = 1.225f;
};

Aerodynamics aero;

struct Battery {
    BatteryType type = BatteryType::LIPO_4S;
    float capacity = 1300.0f;
    float charge = 1300.0f;
    float voltage = 16.8f;           // 4S LiPo fully charged
    float maxVoltage = 16.8f;        // 4.2V * 4 cells
    float nominalVoltage = 14.8f;    // 3.7V * 4 cells
    float minVoltage = 13.2f;        // 3.3V * 4 cells
    float cellCount = 4.0f;
    float internalResistance = 0.01f;
    float sagFactor = 0.1f;
    bool isDepleted = false;

    float getVoltageFromCharge() {
        float chargePercent = charge / capacity;
        float cellMaxVoltage = 4.2f;
        float cellNominalVoltage = 3.7f;
        float cellMinVoltage = 3.3f;

        if (chargePercent > 0.8f) {
            return cellCount * (cellMinVoltage + (cellMaxVoltage - cellMinVoltage) * (chargePercent - 0.8f) / 0.2f);
        }
        else if (chargePercent > 0.2f) {
            return cellCount * (cellMinVoltage + (cellNominalVoltage - cellMinVoltage) * (chargePercent - 0.2f) / 0.6f);
        }
        else {
            return cellCount * cellMinVoltage * chargePercent / 0.2f;
        }
    }

    void configureBattery(BatteryType newType) {
        type = newType;

        // Zellenanzahl basierend auf Batterietyp setzen
        switch (type) {
        case BatteryType::LIPO_1S: cellCount = 1.0f; break;
        case BatteryType::LIPO_2S: cellCount = 2.0f; break;
        case BatteryType::LIPO_3S: cellCount = 3.0f; break;
        case BatteryType::LIPO_4S: cellCount = 4.0f; break;
        case BatteryType::LIPO_5S: cellCount = 5.0f; break;
        case BatteryType::LIPO_6S: cellCount = 6.0f; break;
        }

        maxVoltage = cellCount * 4.2f;
        nominalVoltage = cellCount * 3.7f;
        minVoltage = cellCount * 3.3f;
        voltage = maxVoltage;

        internalResistance = 0.01f * (1.0f + (cellCount - 4.0f) * 0.05f);
        internalResistance = glm::max(0.005f, internalResistance);
    }
};

Battery battery;

const float MS_TO_KMH = 3.6f;
const float RPM_TO_RADS = 0.10472f;

enum class DisplayMode {
    ALL, SIMULATION, BATTERY, MOTORS, CONTROLS, NONE
};

DisplayMode currentDisplayMode = DisplayMode::ALL;
bool showSimulationInfo = true;
bool showBatteryInfo = true;
bool showMotorInfo = true;
bool showControlInfo = true;

glm::vec3 dronePosition(0.0f, 1.0f, 3.0f);
glm::vec3 velocity(0.0f);
glm::quat droneQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
glm::vec3 angularVelocity(0.0f);
glm::vec3 rotationRates(0.0f);

std::string glVersionString;
std::string glslVersionString;

float lastFrameTime = 0.0f;
float deltaTime = 0.0f;
float batteryUpdateTime = 0.0f;
auto simulationStartTime = std::chrono::steady_clock::now();
float previousTotalCurrentDraw = 0.0f;

struct InputState {
    bool throttleUp = false;
    bool throttleDown = false;
    bool pitchForward = false;
    bool pitchBackward = false;
    bool rollLeft = false;
    bool rollRight = false;
    bool yawLeft = false;
    bool yawRight = false;
    bool keyPressed = false;

    // Neue Steuereingaben für Batterietyp
    bool batteryTypeUp = false;    // Höheren Zellentyp wählen
    bool batteryTypeDown = false;  // Niedrigeren Zellentyp wählen

    float pitchInput = 0.0f;
    float rollInput = 0.0f;
    float yawInput = 0.0f;
};

InputState input;

const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
out vec3 ourColor;
uniform mat4 projection;
uniform mat4 view;
void main() {
    gl_Position = projection * view * vec4(aPos, 1.0);
    ourColor = aColor;
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
in vec3 ourColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(ourColor, 1.0);
}
)";

unsigned int shaderProgram;
unsigned int floorVAO, floorVBO, gridVAO, gridVBO, axesVAO, axesVBO;

// Gibt den Namen des Batterietyps als String zurück
std::string getBatteryTypeName(BatteryType type) {
    switch (type) {
    case BatteryType::LIPO_1S: return "1S LiPo";
    case BatteryType::LIPO_2S: return "2S LiPo";
    case BatteryType::LIPO_3S: return "3S LiPo";
    case BatteryType::LIPO_4S: return "4S LiPo";
    case BatteryType::LIPO_5S: return "5S LiPo";
    case BatteryType::LIPO_6S: return "6S LiPo";
    default: return "Unknown";
    }
}

void cycleBatteryTypeUp() {
    BatteryType newType;
    switch (droneConfig.batteryType) {
    case BatteryType::LIPO_1S: newType = BatteryType::LIPO_2S; break;
    case BatteryType::LIPO_2S: newType = BatteryType::LIPO_3S; break;
    case BatteryType::LIPO_3S: newType = BatteryType::LIPO_4S; break;
    case BatteryType::LIPO_4S: newType = BatteryType::LIPO_5S; break;
    case BatteryType::LIPO_5S: newType = BatteryType::LIPO_6S; break;
    case BatteryType::LIPO_6S: newType = BatteryType::LIPO_1S; break;
    default: newType = BatteryType::LIPO_4S; break;
    }
    droneConfig.batteryType = newType;
    droneConfig.minBatteryVoltageForFlight = 3.3f * static_cast<float>(static_cast<int>(newType) + 1);
    battery.configureBattery(newType);
    initializeDrone();
}

void cycleBatteryTypeDown() {
    BatteryType newType;
    switch (droneConfig.batteryType) {
    case BatteryType::LIPO_1S: newType = BatteryType::LIPO_6S; break;
    case BatteryType::LIPO_2S: newType = BatteryType::LIPO_1S; break;
    case BatteryType::LIPO_3S: newType = BatteryType::LIPO_2S; break;
    case BatteryType::LIPO_4S: newType = BatteryType::LIPO_3S; break;
    case BatteryType::LIPO_5S: newType = BatteryType::LIPO_4S; break;
    case BatteryType::LIPO_6S: newType = BatteryType::LIPO_5S; break;
    default: newType = BatteryType::LIPO_4S; break;
    }
    droneConfig.batteryType = newType;
    droneConfig.minBatteryVoltageForFlight = 3.3f * static_cast<float>(static_cast<int>(newType) + 1);
    battery.configureBattery(newType);
    initializeDrone();
}

glm::vec3 getDroneFront() {
    return glm::rotate(droneQuat, glm::vec3(0.0f, 0.0f, -1.0f));
}

glm::vec3 getDroneUp() {
    return glm::rotate(droneQuat, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::vec3 getDroneRight() {
    return glm::rotate(droneQuat, glm::vec3(1.0f, 0.0f, 0.0f));
}

glm::vec3 quatToEuler(const glm::quat& quat) {
    glm::vec3 euler = glm::degrees(glm::eulerAngles(quat));

    if (euler.x > 180.0f) euler.x -= 360.0f;
    if (euler.y > 180.0f) euler.y -= 360.0f;
    if (euler.z > 180.0f) euler.z -= 360.0f;

    return euler;
}

float applyExpo(float input, float expoFactor) {
    return input * (expoFactor * input * input + (1.0f - expoFactor));
}

float calculateMotorEfficiency(float rpm, float throttle) {
    float baseEfficiency = 0.6f + 0.3f * throttle;

    float normalizedRPM = rpm / (motors[0].maxRPM > 0 ? motors[0].maxRPM : 1.0f);
    float rpmEfficiencyFactor = 0.9f;

    if (normalizedRPM < 0.3f) {
        rpmEfficiencyFactor = 0.6f + normalizedRPM;
    }
    else if (normalizedRPM > 0.9f) {
        rpmEfficiencyFactor = 1.0f - (normalizedRPM - 0.9f) * 0.3f;
    }

    return baseEfficiency * rpmEfficiencyFactor;
}

void initializeDrone() {
    float motorOffset = droneConfig.frameSize / 2.0f;

    motors[0].position = glm::vec3(motorOffset, 0.0f, -motorOffset);  // Front-right (CCW)
    motors[1].position = glm::vec3(motorOffset, 0.0f, motorOffset);   // Rear-right (CW)
    motors[2].position = glm::vec3(-motorOffset, 0.0f, motorOffset);  // Rear-left (CCW)
    motors[3].position = glm::vec3(-motorOffset, 0.0f, -motorOffset); // Front-left (CW)

    motors[0].rotationDir = 1.0f;   // CCW
    motors[1].rotationDir = -1.0f;  // CW
    motors[2].rotationDir = 1.0f;   // CCW
    motors[3].rotationDir = -1.0f;  // CW

    battery.capacity = droneConfig.batteryCapacity;
    battery.charge = battery.capacity;
    battery.isDepleted = false;

    // Die Batterie basierend auf dem ausgewählten Typ konfigurieren
    battery.configureBattery(droneConfig.batteryType);

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].maxRPM = battery.maxVoltage * droneConfig.motorKv;
        motors[i].targetThrottle = 0.0f;
        motors[i].currentThrottle = 0.0f;
        motors[i].currentRPM = 0.0f;
        motors[i].targetRPM = 0.0f;
        motors[i].thrust = 0.0f;
        motors[i].torque = 0.0f;
        motors[i].angularVelocity = 0.0f;
        motors[i].isSpinning = false;
        motors[i].currentDraw = 0.0f;
    }

    dronePosition = glm::vec3(0.0f, 1.0f, 3.0f);
    velocity = glm::vec3(0.0f);
    droneQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    angularVelocity = glm::vec3(0.0f);
    rotationRates = glm::vec3(0.0f);

    controls.throttlePosition = 0.0f;
    input = {};

    simulationStartTime = std::chrono::steady_clock::now();
    lastFrameTime = 0.0f;
    deltaTime = 0.0f;
    previousTotalCurrentDraw = 0.0f;
    batteryUpdateTime = 0.0f;
}

void cycleDisplayMode() {
    switch (currentDisplayMode) {
    case DisplayMode::ALL:
        currentDisplayMode = DisplayMode::SIMULATION;
        showSimulationInfo = true;
        showBatteryInfo = false;
        showMotorInfo = false;
        showControlInfo = false;
        break;
    case DisplayMode::SIMULATION:
        currentDisplayMode = DisplayMode::BATTERY;
        showSimulationInfo = false;
        showBatteryInfo = true;
        showMotorInfo = false;
        showControlInfo = false;
        break;
    case DisplayMode::BATTERY:
        currentDisplayMode = DisplayMode::MOTORS;
        showSimulationInfo = false;
        showBatteryInfo = false;
        showMotorInfo = true;
        showControlInfo = false;
        break;
    case DisplayMode::MOTORS:
        currentDisplayMode = DisplayMode::CONTROLS;
        showSimulationInfo = false;
        showBatteryInfo = false;
        showMotorInfo = false;
        showControlInfo = true;
        break;
    case DisplayMode::CONTROLS:
        currentDisplayMode = DisplayMode::NONE;
        showSimulationInfo = false;
        showBatteryInfo = false;
        showMotorInfo = false;
        showControlInfo = false;
        break;
    case DisplayMode::NONE:
        currentDisplayMode = DisplayMode::ALL;
        showSimulationInfo = true;
        showBatteryInfo = true;
        showMotorInfo = true;
        showControlInfo = true;
        break;
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void getOpenGLInfo() {
    const GLubyte* version = glGetString(GL_VERSION);
    if (version) {
        glVersionString = reinterpret_cast<const char*>(version);
    }
    else {
        glVersionString = "Unknown";
    }

    const GLubyte* glslVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);
    if (glslVersion) {
        glslVersionString = reinterpret_cast<const char*>(glslVersion);
    }
    else {
        glslVersionString = "Unknown";
    }
}

unsigned int compileShader(unsigned int type, const char* source) {
    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &source, NULL);
    glCompileShader(id);

    int success;
    char infoLog[512];
    glGetShaderiv(id, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(id, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::" << (type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT")
            << "::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    return id;
}

void setupShaders() {
    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    int success;
    char infoLog[512];
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void setupFloorBuffers() {
    float floorVertices[] = {
        -100.0f, 0.0f, -100.0f, 0.3f, 0.3f, 0.3f,
         100.0f, 0.0f, -100.0f, 0.3f, 0.3f, 0.3f,
         100.0f, 0.0f,  100.0f, 0.3f, 0.3f, 0.3f,
        -100.0f, 0.0f,  100.0f, 0.3f, 0.3f, 0.3f
    };

    glGenVertexArrays(1, &floorVAO);
    glGenBuffers(1, &floorVBO);
    glBindVertexArray(floorVAO);

    glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floorVertices), floorVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    std::vector<float> gridLines;
    float gridColor[] = { 0.5f, 0.5f, 0.5f };
    float gridSpacing = 1.0f;  // Changed from 10.0f to 1.0f for 1x1m grid

    for (float i = -100.0f; i <= 100.0f; i += gridSpacing) {
        gridLines.push_back(i); gridLines.push_back(0.01f); gridLines.push_back(-100.0f);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);

        gridLines.push_back(i); gridLines.push_back(0.01f); gridLines.push_back(100.0f);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);

        gridLines.push_back(-100.0f); gridLines.push_back(0.01f); gridLines.push_back(i);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);

        gridLines.push_back(100.0f); gridLines.push_back(0.01f); gridLines.push_back(i);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);
    }

    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);

    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, gridLines.size() * sizeof(float), gridLines.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    float axesVertices[] = {
        -100.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
         100.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, -100.0f, 0.0f, 0.0f, 1.0f,
         0.0f, 0.0f,  100.0f, 0.0f, 0.0f, 1.0f
    };

    glGenVertexArrays(1, &axesVAO);
    glGenBuffers(1, &axesVBO);
    glBindVertexArray(axesVAO);

    glBindBuffer(GL_ARRAY_BUFFER, axesVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axesVertices), axesVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void drawFloor(const glm::mat4& projection, const glm::mat4& view) {
    glUseProgram(shaderProgram);

    unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
    unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    glBindVertexArray(floorVAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    glBindVertexArray(gridVAO);
    int gridVertexCount = 201 * 2 * 2; // Changed from 21 to 201 for the 1x1m grid (200 lines + 1 center line)
    glDrawArrays(GL_LINES, 0, gridVertexCount);

    glLineWidth(2.0f);
    glBindVertexArray(axesVAO);
    glDrawArrays(GL_LINES, 0, 4);
    glLineWidth(1.0f);

    glBindVertexArray(0);
}

void updateMotors(float deltaTime) {
    float totalCurrentDraw = 0.0f;

    // Sanfter Anlaufstrom für realistischeres Verhalten
    float maxStartupCurrentDraw = 10.0f; // Höherer Anlaufstrom für realistische Leistung
    float currentLimitFactor = glm::min(1.0f, batteryUpdateTime * 2.0f);
    float limitedPreviousCurrentDraw = glm::min(previousTotalCurrentDraw, maxStartupCurrentDraw * currentLimitFactor);

    // Realistischerer Spannungsabfall basierend auf Batterietyp und Last
    float batteryVoltageDrop = battery.sagFactor * battery.internalResistance * limitedPreviousCurrentDraw;
    float effectiveVoltage = std::max(battery.voltage - batteryVoltageDrop, battery.minVoltage * 0.9f);

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].rotationDir = (i % 2 == 0) ? 1.0f : -1.0f;

        float targetThrottle = motors[i].targetThrottle;

        // Realistisches Motoransprechverhalten basierend auf Batteriespannung
        float responseRate;
        if (targetThrottle > motors[i].currentThrottle) {
            responseRate = droneConfig.motorAcceleration * (0.8f + 0.2f * effectiveVoltage / battery.maxVoltage);
        }
        else {
            responseRate = droneConfig.motorDeceleration;
        }

        // Nichtlineare Ansprechkurve für realistischeres Drosselklappenverhalten
        float throttleResponseFactor = 0.6f + 0.4f * motors[i].currentThrottle;
        responseRate *= throttleResponseFactor;

        // Aktuelle Drosselklappe mit angemessener Ansprechrate aktualisieren
        motors[i].currentThrottle = glm::mix(motors[i].currentThrottle, targetThrottle, deltaTime * responseRate);

        // Mindestdrehzahlschwelle für den Motorstart (Überwindung der Haftreibung)
        bool shouldSpin = motors[i].currentThrottle > droneConfig.motorStartupThreshold * 0.8f &&
            !battery.isDepleted && battery.voltage >= droneConfig.minBatteryVoltageForFlight * 0.9f;

        // Motorstart/-stopp-Logik mit Hysterese
        if (shouldSpin && !motors[i].isSpinning && motors[i].currentThrottle > droneConfig.motorStartupThreshold) {
            motors[i].isSpinning = true;
        }
        else if (!shouldSpin && motors[i].isSpinning && motors[i].currentThrottle < droneConfig.motorStartupThreshold * 0.8f) {
            motors[i].isSpinning = false;
        }

        // Ziel-RPM basierend auf Drosselklappe, Spannung und KV-Wert berechnen
        if (motors[i].isSpinning) {
            // Nicht-lineare Throttle-Kurve für realistischere Leistungsentfaltung
            float throttleCurve = powf(motors[i].currentThrottle, 1.1f); // Angepasst für bessere Leistung im unteren Bereich
            motors[i].targetRPM = throttleCurve * effectiveVoltage * droneConfig.motorKv;
        }
        else {
            motors[i].targetRPM = 0.0f;
        }

        // Motoren stoppen, wenn Batterie leer ist
        if (battery.isDepleted || battery.voltage < droneConfig.minBatteryVoltageForFlight * 0.9f) {
            motors[i].targetRPM = 0.0f;
            motors[i].isSpinning = false;
        }

        // Motordrehmoment berechnen
        float maxTorque = 0.00012f * droneConfig.motorKv * effectiveVoltage * motors[i].currentThrottle;

        // Drehmoment-RPM-Kurve
        float torqueRPMFactor = 1.0f - (motors[i].currentRPM / (1.5f * motors[i].maxRPM + 0.001f));
        torqueRPMFactor = glm::clamp(torqueRPMFactor, 0.2f, 1.0f);
        motors[i].torque = maxTorque * torqueRPMFactor;

        // RPM in rad/s umrechnen für physikalische Berechnungen
        float currentRadS = motors[i].currentRPM * RPM_TO_RADS;

        // Winkelbeschleunigung mit Drehmoment und Motorträgheit berechnen
        float angularAcceleration = motors[i].torque / droneConfig.motorInertia;

        // Reibung/Widerstand anwenden
        float rotationalFriction = 0.000008f * motors[i].currentRPM;
        angularAcceleration -= rotationalFriction * currentRadS / droneConfig.motorInertia;

        // Winkelgeschwindigkeit aktualisieren
        motors[i].angularVelocity += angularAcceleration * deltaTime;
        motors[i].angularVelocity = glm::max(0.0f, motors[i].angularVelocity);

        // Zurück zu RPM umrechnen
        motors[i].currentRPM = motors[i].angularVelocity / RPM_TO_RADS;

        // Schubberechnung nach der Propellertheorie:
        // T = CT * rho * n^2 * D^4
        float rps = motors[i].currentRPM / 60.0f;  // Umdrehungen pro Sekunde
        float propDiameterM = droneConfig.propSize * 0.0254f;  // Zoll zu Meter

        // Verbesserte Schubberechnung
        float thrust_coefficient = droneConfig.thrustCoefficient;

        // Der Exponent von 2.3 ist realistischer für moderne Propeller
        float rpm_factor = powf(rps, 2.3f);
        motors[i].thrust = thrust_coefficient * aero.airDensity * rpm_factor *
            powf(propDiameterM, 4.0f) * droneConfig.propEfficiency;

        // Deutlich verbesserte Spannungseffizienz
        float voltageEfficiencyFactor = glm::clamp(effectiveVoltage / battery.nominalVoltage, 0.8f, 1.8f);
        motors[i].thrust *= voltageEfficiencyFactor;

        // Füge eine Boost-Funktion für niedrige RPMs hinzu - simuliert das Ansprechverhalten moderner ESCs
        if (motors[i].currentRPM < motors[i].maxRPM * 0.3f && motors[i].targetThrottle > 0.15f) {
            motors[i].thrust *= 1.2f; // Extra Schub bei niedrigen RPMs
        }

        // In updateMotors, ca. Zeile 425-433 ändern:

        // Schubverlust bei hoher Geschwindigkeit reduzieren/verbessern
        float forwardSpeed = glm::length(velocity);
        if (forwardSpeed > 8.0f) { // Von 5.0 auf 8.0 erhöhen - höhere Geschwindigkeit vor Stalleffekt
            // Weniger Schubverlust bei höherer Geschwindigkeit
            float stallFactor = 1.0f - glm::clamp((forwardSpeed - 8.0f) / 40.0f, 0.0f, 0.25f);
            motors[i].thrust *= stallFactor;
        }

        // Leistungs- und Stromberechnung
        float motorPower = motors[i].torque * motors[i].angularVelocity;
        float motorEfficiency = calculateMotorEfficiency(motors[i].currentRPM, motors[i].currentThrottle);

        if (effectiveVoltage * motorEfficiency > 0.001f) {
            motors[i].currentDraw = motorPower / (effectiveVoltage * motorEfficiency);
        }
        else {
            motors[i].currentDraw = 0.0f;
        }

        totalCurrentDraw += motors[i].currentDraw;
    }

    // Stromänderungsrate limitieren für realistischeres Verhalten
    float maxCurrentRate = 15.0f;
    float smoothedTotalCurrentDraw = previousTotalCurrentDraw;

    if (totalCurrentDraw > previousTotalCurrentDraw) {
        smoothedTotalCurrentDraw += glm::min(totalCurrentDraw - previousTotalCurrentDraw, maxCurrentRate * deltaTime);
    }
    else {
        smoothedTotalCurrentDraw = totalCurrentDraw;
    }

    // Batterieverbrauch simulieren
    float hoursElapsed = deltaTime / 3600.0f;
    float batteryDrainFactor = 0.85f; // Realistischer Faktor für die Entladungsrate

    battery.charge -= smoothedTotalCurrentDraw * hoursElapsed * 1000.0f * batteryDrainFactor;
    battery.charge = std::max(0.0f, battery.charge);

    // Batteriespannung aktualisieren
    float loadChangeFactor = 1.0f;
    if (totalCurrentDraw < previousTotalCurrentDraw) {
        loadChangeFactor = 2.0f;  // Schnellere Erholung bei Lastabnahme
    }

    float baseVoltage = battery.getVoltageFromCharge();
    float sagAmount = battery.sagFactor * battery.internalResistance * smoothedTotalCurrentDraw / loadChangeFactor;
    battery.voltage = glm::max(baseVoltage - sagAmount, battery.minVoltage * 0.95f);

    previousTotalCurrentDraw = smoothedTotalCurrentDraw;

    // Batterieleer-Flag setzen
    if (battery.charge <= 1.0f || battery.voltage <= battery.minVoltage) {
        battery.isDepleted = true;
    }

    batteryUpdateTime += deltaTime;
}

void setMotorThrottles(float mainThrottle, const glm::vec3& rotationRates) {
    float pitchEffect = rotationRates.x * 0.2f;
    float rollEffect = rotationRates.z * 0.2f;
    float yawEffect = rotationRates.y * 0.15f;  // Erhöhter Yaw-Effekt für bessere Steuerbarkeit

    motors[0].targetThrottle = mainThrottle - pitchEffect - rollEffect + yawEffect; // Front-right
    motors[1].targetThrottle = mainThrottle + pitchEffect - rollEffect - yawEffect; // Rear-right
    motors[2].targetThrottle = mainThrottle + pitchEffect + rollEffect + yawEffect; // Rear-left
    motors[3].targetThrottle = mainThrottle - pitchEffect + rollEffect - yawEffect; // Front-left

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].targetThrottle = glm::clamp(motors[i].targetThrottle, 0.0f, maxThrottle);
    }

    // Extra Schub beim Beschleunigen für realistischeres Flugverhalten
    static float prevThrottle = 0.0f;
    if (mainThrottle > prevThrottle + 0.02f) {
        // Kurzer Schub-Boost beim Gasgeben
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].targetThrottle = glm::min(motors[i].targetThrottle * 1.1f, maxThrottle);
        }
    }
    prevThrottle = mainThrottle;
}

void updateInputState(GLFWwindow* window) {
    input.throttleUp = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    input.throttleDown = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;

    input.pitchForward = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    input.pitchBackward = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    input.rollLeft = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    input.rollRight = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    input.yawLeft = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    input.yawRight = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;

    // Batterietyp-Steuerung
    input.batteryTypeUp = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
    input.batteryTypeDown = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;

    input.keyPressed = input.throttleUp || input.throttleDown ||
        input.pitchForward || input.pitchBackward ||
        input.rollLeft || input.rollRight ||
        input.yawLeft || input.yawRight ||
        input.batteryTypeUp || input.batteryTypeDown;
}

void processInput(GLFWwindow* window) {
    float currentTime = static_cast<float>(glfwGetTime());
    deltaTime = currentTime - lastFrameTime;
    lastFrameTime = currentTime;

    if (deltaTime > 0.05f) deltaTime = 0.05f;  // Deltazeit begrenzen

    static float lastKeyPressTime = 0.0f;
    float keyDebounceTime = 0.3f;

    if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        cycleDisplayMode();
        lastKeyPressTime = currentTime;
    }

    if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        showBatteryInfo = !showBatteryInfo;
        lastKeyPressTime = currentTime;
    }

    if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        showMotorInfo = !showMotorInfo;
        lastKeyPressTime = currentTime;
    }

    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        showControlInfo = !showControlInfo;
        lastKeyPressTime = currentTime;
    }

    // Batterietyp wechseln
    if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        cycleBatteryTypeUp();
        lastKeyPressTime = currentTime;
    }

    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        cycleBatteryTypeDown();
        lastKeyPressTime = currentTime;
    }

    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
        initializeDrone();
    }

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    updateInputState(window);

    if (input.throttleUp && !input.throttleDown) {
        controls.throttlePosition += controls.throttleIncrement * deltaTime;
    }
    else if (input.throttleDown && !input.throttleUp) {
        controls.throttlePosition -= controls.throttleIncrement * deltaTime;
    }
    else if (controls.throttleReturnToZero) {
        controls.throttlePosition = glm::mix(controls.throttlePosition, 0.0f, deltaTime * controls.throttleResponseRate);
    }

    controls.throttlePosition = glm::clamp(controls.throttlePosition, 0.0f, 1.0f);
    float mainThrottle = controls.throttlePosition * maxThrottle;

    glm::vec3 rotationInput(0.0f);

    if (input.pitchForward && !input.pitchBackward) {
        input.pitchInput = glm::mix(input.pitchInput, -1.0f, deltaTime * 5.0f);
    }
    else if (input.pitchBackward && !input.pitchForward) {
        input.pitchInput = glm::mix(input.pitchInput, 1.0f, deltaTime * 5.0f);
    }
    else {
        input.pitchInput = glm::mix(input.pitchInput, 0.0f, deltaTime * 8.0f);
    }

    if (input.rollLeft && !input.rollRight) {
        input.rollInput = glm::mix(input.rollInput, 1.0f, deltaTime * 5.0f);
    }
    else if (input.rollRight && !input.rollLeft) {
        input.rollInput = glm::mix(input.rollInput, -1.0f, deltaTime * 5.0f);
    }
    else {
        input.rollInput = glm::mix(input.rollInput, 0.0f, deltaTime * 8.0f);
    }

    if (input.yawLeft && !input.yawRight) {
        input.yawInput = glm::mix(input.yawInput, 1.0f, deltaTime * 5.0f);
    }
    else if (input.yawRight && !input.yawLeft) {
        input.yawInput = glm::mix(input.yawInput, -1.0f, deltaTime * 5.0f);
    }
    else {
        input.yawInput = glm::mix(input.yawInput, 0.0f, deltaTime * 8.0f);
    }

    float expoFactor = 0.6f;
    rotationInput.x = applyExpo(input.pitchInput, expoFactor);
    rotationInput.z = applyExpo(input.rollInput, expoFactor);
    rotationInput.y = applyExpo(input.yawInput, expoFactor);

    rotationInput *= controls.rotationSensitivity;

    float voltageRatio = glm::clamp((battery.voltage - battery.minVoltage) /
        (battery.maxVoltage - battery.minVoltage), 0.6f, 1.0f);

    float responseRate = 5.0f * voltageRatio;
    rotationRates = glm::mix(rotationRates, rotationInput * controls.maxRotationRate * voltageRatio, deltaTime * responseRate);
    rotationRates *= (1.0f - rotationalDrag * deltaTime);

    setMotorThrottles(mainThrottle, rotationRates);
    updateMotors(deltaTime);

    glm::mat3 rotMatrix = glm::mat3_cast(droneQuat);
    glm::vec3 bodyAngularVelocity = glm::vec3(
        rotationRates.x,  // Pitch
        rotationRates.y,  // Yaw
        rotationRates.z   // Roll
    );

    angularVelocity = rotMatrix * bodyAngularVelocity;

    float angularVelocityMagnitude = glm::length(angularVelocity);
    glm::quat deltaQuat(1.0f, 0.0f, 0.0f, 0.0f);

    if (angularVelocityMagnitude > 0.001f) {
        float angle = angularVelocityMagnitude * deltaTime;
        glm::vec3 axis = angularVelocity / angularVelocityMagnitude;
        deltaQuat = glm::angleAxis(angle, axis);
        droneQuat = glm::normalize(deltaQuat * droneQuat);
    }

    glm::vec3 droneUp = getDroneUp();
    glm::vec3 droneFront = getDroneFront();
    glm::vec3 droneRight = getDroneRight();

    glm::vec3 thrustForce(0.0f);
    for (int i = 0; i < NUM_MOTORS; i++) {
        thrustForce += droneUp * motors[i].thrust;
    }

    // Gravity force stärker reduzieren für realistischeres Flugverhalten
    glm::vec3 gravityForceVec = glm::vec3(0.0f, -gravityForce * droneConfig.mass * 0.93f, 0.0f);

    // Und verbessere den Ground Effect erheblich:
    float groundEffectMultiplier = 1.0f;
    if (dronePosition.y < aero.groundEffectHeight * 1.2f) {
        groundEffectMultiplier = 1.0f + aero.groundEffectStrength * 3.0f *
            (1.0f - dronePosition.y / (aero.groundEffectHeight * 1.2f));
        thrustForce.y *= groundEffectMultiplier;
    }

    glm::vec3 propWash(0.0f);
    float totalThrust = 0.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        totalThrust += motors[i].thrust;
    }

    if (totalThrust > 0.1f) {
        float turbulence = aero.propWashEffect * totalThrust / 4.0f;
        propWash = glm::vec3(
            ((float)rand() / RAND_MAX * 2.0f - 1.0f) * turbulence,
            ((float)rand() / RAND_MAX * 2.0f - 1.0f) * turbulence * 0.5f,
            ((float)rand() / RAND_MAX * 2.0f - 1.0f) * turbulence
        );
    }

    glm::vec3 dragForce(0.0f);
    if (glm::length(velocity) > 0.001f) {
        float frontalArea = droneConfig.frameSize * droneConfig.frameSize * 0.5f;
        float dynamicPressure = 0.5f * aero.airDensity * glm::length(velocity) * glm::length(velocity);

        float forwardDrag = glm::abs(glm::dot(velocity, droneFront)) * dynamicPressure *
            frontalArea * aero.dragCoefficients.z;
        float sideDrag = glm::abs(glm::dot(velocity, droneRight)) * dynamicPressure *
            frontalArea * aero.dragCoefficients.x;
        float verticalDrag = glm::abs(glm::dot(velocity, droneUp)) * dynamicPressure *
            frontalArea * aero.dragCoefficients.y;

        dragForce = -glm::normalize(velocity) * (forwardDrag + sideDrag + verticalDrag);
    }

    glm::vec3 gyroscopicTorque(0.0f);
    for (int i = 0; i < NUM_MOTORS; i++) {
        float angularMomentum = droneConfig.motorInertia * motors[i].angularVelocity;
        glm::vec3 spinAxis = droneUp * motors[i].rotationDir;
        glm::vec3 motorAngularMomentum = spinAxis * angularMomentum;
        gyroscopicTorque += glm::cross(angularVelocity, motorAngularMomentum);
    }

    float gyroScaleFactor = 0.03f;
    glm::vec3 gyroscopicAcceleration = gyroscopicTorque * gyroScaleFactor / droneConfig.mass / droneConfig.frameSize;
    angularVelocity += gyroscopicAcceleration * deltaTime;

    glm::vec3 totalForce = thrustForce + gravityForceVec + dragForce + propWash;

    // Verbesserte Highspeed-Leistung
    if (glm::length(velocity) > 10.0f) {
        // Bei hoher Geschwindigkeit Luftwiderstand verringern und bessere Aerodynamik simulieren
        float speedFactor = glm::min(glm::length(velocity) / 20.0f, 1.0f);
        dragForce *= (1.0f - 0.3f * speedFactor); // Reduzierter Luftwiderstand bei hoher Geschwindigkeit

        // Vorwärtstrimm für bessere Geschwindigkeit - Drohne neigt sich automatisch etwas nach vorn
        if (glm::length(glm::vec2(velocity.x, velocity.z)) > 15.0f && glm::abs(velocity.y) < 3.0f) {
            glm::vec3 forwardTilt = glm::vec3(-0.2f, 0.0f, 0.0f) * speedFactor;
            rotationRates = glm::mix(rotationRates, forwardTilt, deltaTime * 0.8f);
        }
    }

    glm::vec3 acceleration = totalForce / droneConfig.mass;
    velocity += acceleration * deltaTime;
    dronePosition += velocity * deltaTime;

    if (dronePosition.y < 0.1f) {
        dronePosition.y = 0.1f;

        float impactVelocity = -velocity.y;
        float horizontalVelocity = glm::length(glm::vec2(velocity.x, velocity.z));

        if (impactVelocity > 2.0f || horizontalVelocity > 3.0f) {
            velocity.y = impactVelocity * 0.3f;
            velocity.x *= 0.7f;
            velocity.z *= 0.7f;

            float impactForce = impactVelocity + horizontalVelocity;
            angularVelocity += glm::vec3(
                ((float)rand() / RAND_MAX * 2.0f - 1.0f),
                ((float)rand() / RAND_MAX * 2.0f - 1.0f),
                ((float)rand() / RAND_MAX * 2.0f - 1.0f)
            ) * impactForce * 0.2f;
        }
        else {
            if (velocity.y < 0) velocity.y = 0.0f;

            float groundFriction = 0.1f;
            velocity.x *= (1.0f - groundFriction);
            velocity.z *= (1.0f - groundFriction);

            if (glm::length(glm::vec2(velocity.x, velocity.z)) < 0.05f) {
                velocity.x = 0.0f;
                velocity.z = 0.0f;
            }
        }
    }
}

void updateWindowTitle(GLFWwindow* window, float fps) {
    std::stringstream title;
    glm::vec3 eulerAngles = quatToEuler(droneQuat);

    float speedMS = glm::length(velocity);
    float speedKMH = speedMS * MS_TO_KMH;

    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsedSeconds = currentTime - simulationStartTime;
    int elapsedMinutes = static_cast<int>(elapsedSeconds.count()) / 60;
    int elapsedSecondsRemainder = static_cast<int>(elapsedSeconds.count()) % 60;

    float avgThrottle = 0.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        avgThrottle += motors[i].currentThrottle;
    }
    avgThrottle /= NUM_MOTORS;

    float batteryPercentage = (battery.charge / battery.capacity) * 100.0f;

    title << "FPV Drone Sim | ";
    title << "FPS: " << std::fixed << std::setprecision(1) << fps << " | ";
    title << getBatteryTypeName(droneConfig.batteryType) << " | ";

    if (showSimulationInfo) {
        title << "Speed: " << std::setprecision(1) << speedKMH << " km/h | "
            << "Alt: " << std::setprecision(1) << dronePosition.y << "m | ";
    }

    if (showControlInfo) {
        title << "Throttle: " << std::setprecision(2) << controls.throttlePosition << " | ";
    }

    if (showBatteryInfo) {
        title << "Batt: " << std::setprecision(1) << batteryPercentage << "% | "
            << std::setprecision(1) << battery.voltage << "V | ";

        if (battery.isDepleted) {
            title << "[EMPTY] | ";
        }

        title << "Flight: " << elapsedMinutes << ":" << std::setw(2) << std::setfill('0')
            << elapsedSecondsRemainder << " | ";
    }

    if (showMotorInfo) {
        title << "Motors: ";
        for (int i = 0; i < NUM_MOTORS; i++) {
            title << std::setprecision(1) << motors[i].currentRPM / 1000.0f << "K ";
        }
        title << "| ";
    }

    title << "W/S: Throttle | Arrow: Pitch/Roll | A/D: Yaw | N/P: Batt Type | R: Reset";

    glfwSetWindowTitle(window, title.str().c_str());
}

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "FPV Drone Simulator", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    getOpenGLInfo();
    std::cout << "OpenGL Version: " << glVersionString << std::endl;
    std::cout << "GLSL Version: " << glslVersionString << std::endl;

    srand(static_cast<unsigned int>(time(nullptr)));
    initializeDrone();
    setupShaders();
    setupFloorBuffers();

    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) {
        processInput(window);

        float fps = 1.0f / (deltaTime > 0.0001f ? deltaTime : 0.0001f);

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(glm::radians(75.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 500.0f);

        glm::vec3 cameraOffset = glm::rotate(droneQuat, glm::vec3(0.0f, 0.05f, -0.15f));
        glm::vec3 cameraPosition = dronePosition + cameraOffset;

        glm::mat4 cameraRotation = glm::mat4_cast(droneQuat);
        glm::mat4 cameraTransform = glm::translate(glm::mat4(1.0f), cameraPosition) * cameraRotation;
        glm::mat4 view = glm::inverse(cameraTransform);

        drawFloor(projection, view);
        updateWindowTitle(window, fps);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &floorVAO);
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteVertexArrays(1, &axesVAO);
    glDeleteBuffers(1, &floorVBO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteBuffers(1, &axesVBO);
    glDeleteProgram(shaderProgram);

    glfwTerminate();
    return 0;
}