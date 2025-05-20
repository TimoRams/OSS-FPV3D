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

// ImGui includes
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// Constants
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
const float PI = 3.14159265358979323846f;
const float MS_TO_KMH = 3.6f;
const float RPM_TO_RADS = 0.10472f;
const float INCH_TO_METER = 0.0254f;
const float GRAVITY = 9.81f;
const float MAX_THROTTLE = 1.0f;
const float ROTATIONAL_DRAG = 0.3f;
const int NUM_MOTORS = 4;

// Global state variables
int currentWidth = SCR_WIDTH;
int currentHeight = SCR_HEIGHT;
float lastFrameTime = 0.0f;
float deltaTime = 0.0f;
float batteryUpdateTime = 0.0f;
auto simulationStartTime = std::chrono::steady_clock::now();
float previousTotalCurrentDraw = 0.0f;
float cameraAngle = 0.0f;
const float CAMERA_ANGLE_INCREMENT = 5.0f;
bool armed = false;

// OpenGL related variables
std::string glVersionString;
std::string glslVersionString;
unsigned int shaderProgram;
unsigned int floorVAO, floorVBO, gridVAO, gridVBO, axesVAO, axesVBO, skyboxVAO, skyboxVBO;

// Battery types
enum class BatteryType {
    LIPO_1S,
    LIPO_2S,
    LIPO_3S,
    LIPO_4S,
    LIPO_5S,
    LIPO_6S
};

// Display mode for UI
enum class DisplayMode {
    ALL,
    SIMULATION,
    BATTERY,
    MOTORS,
    CONTROLS,
    NONE
};

DisplayMode currentDisplayMode = DisplayMode::ALL;
bool showSimulationInfo = true;
bool showBatteryInfo = true;
bool showMotorInfo = true;
bool showControlInfo = true;

// Forward declarations
void initializeDrone();
void updateMotors(float deltaTime);
void setMotorThrottles(float mainThrottle, const glm::vec3& rotationRates);
void updateGroundContacts();
bool isDroneOnGround();
void handleGroundCollision();
std::string getBatteryTypeName(BatteryType type);

// Control settings structure
struct ControlSettings {
    float rotationSensitivity = 1.2f;
    float maxRotationRate = glm::radians(1200.0f);
    float throttleIncrement = 0.4f;
    float throttlePosition = 0.0f;
    float throttleResponseRate = 3.0f;
    bool throttleReturnToZero = false;
};

ControlSettings controls;

// Drone configuration structure
struct DroneConfig {
    // Basic physical properties
    float mass = 0.52f;                      // Weight in kg
    float frameSize = 0.23f;                 // Frame size in meters (diagonal motor-to-motor)
    float propSize = 5.0f;                   // Propeller size in inches

    // Performance characteristics
    float thrustCoefficient = 0.52f;         // Thrust coefficient (affects lift efficiency)
    float propEfficiency = 0.95f;            // Propeller efficiency factor (0-1)

    // Motor specifications
    float motorKv = 5300.0f;                 // Motor KV rating (RPM per volt)
    float motorResponseRate = 16.0f;         // How quickly motors respond to throttle changes
    float motorAcceleration = 50.0f;         // Motor acceleration rate
    float motorDeceleration = 10.0f;         // Motor deceleration rate
    float motorInertia = 0.000018f;          // Motor rotational inertia
    float motorStartupThreshold = 0.05f;     // Minimum throttle required to start motors

    // Power system
    BatteryType batteryType = BatteryType::LIPO_4S;  // Default battery type
    float batteryCapacity = 1500.0f;                 // Battery capacity in mAh
    float minBatteryVoltageForFlight = 3.3f * 4.0f;  // Minimum safe voltage

    // Helper method to configure a drone with standard presets
    void applyPreset(const std::string& presetName) {
        if (presetName == "micro") {
            // 3" micro quad preset
            frameSize = 0.12f;    // 120mm
            mass = 0.25f;         // 250g
            propSize = 3.0f;      // 3" props
            motorKv = 7500.0f;    // Higher KV for small props
            thrustCoefficient = 0.48f;
            batteryType = BatteryType::LIPO_3S;
            batteryCapacity = 450.0f;
            motorInertia = 0.000010f;
        }
        else if (presetName == "toothpick") {
            // Toothpick style 2.5-3"
            frameSize = 0.10f;    // 100mm
            mass = 0.15f;         // 150g
            propSize = 2.5f;      // 2.5" props
            motorKv = 11000.0f;   // Very high KV
            thrustCoefficient = 0.42f;
            batteryType = BatteryType::LIPO_2S;
            batteryCapacity = 350.0f;
            motorInertia = 0.000008f;
        }
        else if (presetName == "cinematic") {
            // Cinematic 5" quad
            frameSize = 0.23f;    // 230mm
            mass = 0.65f;         // 650g
            propSize = 5.1f;      // 5.1" props
            motorKv = 1900.0f;    // Low KV for smoother flying
            thrustCoefficient = 0.55f;
            batteryType = BatteryType::LIPO_6S;
            batteryCapacity = 1500.0f;
            motorResponseRate = 12.0f;  // Slower for smoother movement
            motorInertia = 0.000022f;
        }
        else if (presetName == "freestyle") {
            // Standard 5" freestyle
            frameSize = 0.23f;    // 230mm
            mass = 0.52f;         // 520g
            propSize = 5.0f;      // 5" props
            motorKv = 2550.0f;    // Balanced KV
            thrustCoefficient = 0.54f;
            batteryType = BatteryType::LIPO_4S;
            batteryCapacity = 1300.0f;
            motorResponseRate = 16.0f;
        }
        else if (presetName == "racing") {
            // Racing 5" quad
            frameSize = 0.22f;    // 220mm
            mass = 0.42f;         // 420g (lightweight)
            propSize = 5.1f;      // 5.1" props
            motorKv = 2750.0f;    // Higher KV for speed
            thrustCoefficient = 0.58f;
            batteryType = BatteryType::LIPO_4S;
            batteryCapacity = 1100.0f;  // Lighter battery
            motorAcceleration = 60.0f;  // Faster response
            motorInertia = 0.000017f;
        }
        else if (presetName == "long-range") {
            // 7" long range quad
            frameSize = 0.30f;    // 300mm
            mass = 0.70f;         // 700g
            propSize = 7.0f;      // 7" props
            motorKv = 1700.0f;    // Lower KV for efficiency
            thrustCoefficient = 0.56f;
            propEfficiency = 0.97f;
            batteryType = BatteryType::LIPO_6S;
            batteryCapacity = 3000.0f;  // Larger battery
            motorResponseRate = 14.0f;
        }

        // Update minimum battery voltage based on cell count
        updateMinBatteryVoltage();
    }

    // Update minimum battery voltage based on current battery type
    void updateMinBatteryVoltage() {
        int cellCount = static_cast<int>(batteryType) + 1;
        minBatteryVoltageForFlight = 3.3f * static_cast<float>(cellCount);
    }

    // Calculate maximum theoretical thrust with current configuration
    float calculateMaxThrust() const {
        // Based on battery cells, motor KV, and prop size
        int cells = static_cast<int>(batteryType) + 1;
        float maxVoltage = cells * 4.2f;  // Full battery voltage
        float maxRPM = maxVoltage * motorKv;
        float propDiameterM = propSize * INCH_TO_METER;

        // Rough thrust calculation: kg of thrust per motor at 100% throttle
        float airDensity = 1.225f; // kg/m³
        float rps = maxRPM / 60.0f;
        float rpm_factor = powf(rps, 2.3f);

        return thrustCoefficient * airDensity * rpm_factor *
            powf(propDiameterM, 4.0f) * propEfficiency;
    }

    // Get thrust-to-weight ratio
    float getThrustToWeightRatio() const {
        float maxThrustPerMotor = calculateMaxThrust();
        float totalThrust = maxThrustPerMotor * 4.0f; // Assuming quadcopter with 4 motors
        return totalThrust / (mass * GRAVITY);
    }
};

DroneConfig droneConfig;

// Motor structure
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

// Aerodynamics structure
struct Aerodynamics {
    glm::vec3 dragCoefficients = glm::vec3(0.2f, 0.3f, 0.2f);
    float groundEffectHeight = 0.5f;
    float groundEffectStrength = 1.3f;
    float propWashEffect = 0.12f;
    float airDensity = 1.225f;
};

Aerodynamics aero;

// Battery structure
struct Battery {
    BatteryType type = BatteryType::LIPO_4S;
    float capacity = 1300.0f;
    float charge = 1300.0f;
    float voltage = 16.8f;
    float maxVoltage = 16.8f;
    float nominalVoltage = 14.8f;
    float minVoltage = 13.2f;
    float cellCount = 4.0f;
    float internalResistance = 0.01f;
    float sagFactor = 0.1f;
    bool isDepleted = false;

    float baseBatteryVoltage = 0.0f;  // Voltage based on current charge level (without sag)
    float currentVoltageSag = 0.0f;   // Current voltage sag from load
    float lastLoadLevel = 0.0f;       // Last load level for recovery calculation

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

// Input state structure
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
    bool batteryTypeUp = false;
    bool batteryTypeDown = false;
    bool armToggle = false;

    float pitchInput = 0.0f;
    float rollInput = 0.0f;
    float yawInput = 0.0f;
};

InputState input;

// Frame size and dimensions
struct DroneFrameSize {
    // Basic dimensions in inches
    float frameSizeInches = 9.0f;    // Frame size in inches (9" frame = 230mm)
    float frameHeightInches = 1.2f;  // Frame height in inches
    float propGuardHeightInches = 0.5f; // Additional height for prop guards
    float propSizeInches = 5.0f;     // Propeller size in inches
    float armWidthInches = 0.6f;     // Arm width in inches

    // Calculated metric values
    float frameSizeMeters = 0.0f;    // Will be calculated from inches
    float frameHeightMeters = 0.0f;  // Will be calculated from inches
    float propGuardHeightMeters = 0.0f; // Will be calculated from inches
    float propSizeMeters = 0.0f;     // Will be calculated from inches
    float armWidthMeters = 0.0f;     // Will be calculated from inches

    // Drone dimensions (for collision bounding box)
    float width = 0.0f;   // Total width including props
    float length = 0.0f;  // Total length including props
    float height = 0.0f;  // Total height including props and guards

    // Collision properties
    float elasticity = 0.05f;  // Very low elasticity (lower = less bounce)
    float friction = 0.95f;    // High friction (higher = less sliding)

    // Additional contact points for better ground interaction
    static const int NUM_CONTACT_POINTS = 9;  // 4 corners + 4 motor mounts + center
    glm::vec3 contactPoints[NUM_CONTACT_POINTS]; // Local space positions
    float contactPointMass[NUM_CONTACT_POINTS]; // Distribution of mass at each point

    void updateFromConfig(const DroneConfig& config) {
        // Update frame size in metric from frameSize in config (which is in meters)
        frameSizeInches = config.frameSize * 39.37f; // Convert meters to inches
        propSizeInches = config.propSize;           // Already in inches

        // Convert all inch values to meters
        frameSizeMeters = frameSizeInches * INCH_TO_METER;
        frameHeightMeters = frameHeightInches * INCH_TO_METER;
        propGuardHeightMeters = propGuardHeightInches * INCH_TO_METER;
        propSizeMeters = propSizeInches * INCH_TO_METER;
        armWidthMeters = armWidthInches * INCH_TO_METER;

        // Calculate overall dimensions including props
        width = frameSizeMeters + propSizeMeters * 0.5f;  // Full width including props
        length = frameSizeMeters + propSizeMeters * 0.5f; // Full length including props
        height = frameHeightMeters + propGuardHeightMeters; // Total height

        // Recalculate all contact points in local space
        float halfSize = frameSizeMeters / 2.0f;
        float bottomHeight = -frameHeightMeters / 2.0f;

        // Four corners of frame
        contactPoints[0] = glm::vec3(halfSize, bottomHeight, -halfSize);  // Front-right
        contactPoints[1] = glm::vec3(halfSize, bottomHeight, halfSize);   // Back-right
        contactPoints[2] = glm::vec3(-halfSize, bottomHeight, halfSize);  // Back-left
        contactPoints[3] = glm::vec3(-halfSize, bottomHeight, -halfSize); // Front-left

        // Motor mount positions (slightly inset from corners)
        float motorInset = halfSize * 0.1f;
        contactPoints[4] = glm::vec3(halfSize - motorInset, bottomHeight, -halfSize + motorInset);  // Front-right motor
        contactPoints[5] = glm::vec3(halfSize - motorInset, bottomHeight, halfSize - motorInset);   // Back-right motor
        contactPoints[6] = glm::vec3(-halfSize + motorInset, bottomHeight, halfSize - motorInset);  // Back-left motor
        contactPoints[7] = glm::vec3(-halfSize + motorInset, bottomHeight, -halfSize + motorInset); // Front-left motor

        // Center of frame
        contactPoints[8] = glm::vec3(0.0f, bottomHeight, 0.0f);

        // Set mass distribution (used for calculating impact responses)
        // Center has more weight
        contactPointMass[8] = 0.3f;  // Center - 30% of mass

        // Corners have less weight
        for (int i = 0; i < 4; i++) {
            contactPointMass[i] = 0.075f; // Each corner - 7.5% of mass (30% total)
        }

        // Motor mounts have more weight (motors and electronics)
        for (int i = 4; i < 8; i++) {
            contactPointMass[i] = 0.1f; // Each motor - 10% of mass (40% total)
        }
    }

    // Get frame size as a formatted string
    std::string getSizeString() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << frameSizeInches << "\" Frame / "
            << propSizeInches << "\" Props";
        return ss.str();
    }
};

DroneFrameSize droneFrame;

// Ground contact detection structure
struct DroneGroundContact {
    static const int MAX_CONTACTS = DroneFrameSize::NUM_CONTACT_POINTS;

    bool inContact[MAX_CONTACTS];        // Whether each point is in contact with ground
    float groundDistances[MAX_CONTACTS]; // Distance of each point from ground
    glm::vec3 worldPoints[MAX_CONTACTS]; // Contact points in world space
    float penetrationDepths[MAX_CONTACTS]; // How deep each point is penetrating the ground
    glm::vec3 impactNormals[MAX_CONTACTS]; // Surface normal at contact point
    float impactForces[MAX_CONTACTS];    // Force of impact at each point

    int totalContactsCount = 0;          // Number of points currently in contact
    float maxPenetrationDepth = 0.0f;    // Maximum penetration depth
    bool isGrounded = false;             // Whether the drone is considered grounded

    // Collision detection optimization
    glm::vec3 boundingBoxMin;            // Minimum point of bounding box in world space
    glm::vec3 boundingBoxMax;            // Maximum point of bounding box in world space
    bool boundingBoxUpdated = false;     // Whether the bounding box needs updating

    // For stability calculation
    float leftRightBalance = 0.0f;       // -1 = all left, +1 = all right
    float frontBackBalance = 0.0f;       // -1 = all front, +1 = all back
    bool isStable = false;               // Whether the drone is in a stable configuration

    // Reset the ground contact state
    void reset() {
        totalContactsCount = 0;
        maxPenetrationDepth = 0.0f;
        isGrounded = false;
        leftRightBalance = 0.0f;
        frontBackBalance = 0.0f;
        isStable = false;
        boundingBoxUpdated = false;

        for (int i = 0; i < MAX_CONTACTS; i++) {
            inContact[i] = false;
            groundDistances[i] = 1000.0f; // Large initial distance
            penetrationDepths[i] = 0.0f;
            impactForces[i] = 0.0f;
            impactNormals[i] = glm::vec3(0.0f, 1.0f, 0.0f); // Default to up
        }
    }

    // Update the bounding box for spatial optimization
    void updateBoundingBox(const glm::vec3& position, const DroneFrameSize& frameSize) {
        // Calculate the bounding box based on drone position and size
        float halfWidth = frameSize.width / 2.0f;
        float halfLength = frameSize.length / 2.0f;
        float halfHeight = frameSize.height / 2.0f;

        boundingBoxMin = position - glm::vec3(halfWidth, halfHeight, halfLength);
        boundingBoxMax = position + glm::vec3(halfWidth, halfHeight, halfLength);
        boundingBoxUpdated = true;
    }

    // Quick check if the drone might be in contact with the ground
    bool quickGroundCheck() const {
        // If the bottom of the bounding box is near or below ground level
        return boundingBoxMin.y <= 0.1f;
    }
};

DroneGroundContact groundContact;

// Shader sources
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

// Game state
struct JoystickState {
    ImVec2 leftStick = ImVec2(0.5f, 0.5f);   // Left stick (throttle/yaw): x=yaw, y=throttle
    ImVec2 rightStick = ImVec2(0.5f, 0.5f);  // Right stick (pitch/roll): x=roll, y=pitch
};

JoystickState joystickState;

// Physics state
glm::vec3 dronePosition(0.0f, 1.0f, 3.0f);
glm::vec3 velocity(0.0f);
glm::quat droneQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
glm::vec3 angularVelocity(0.0f);
glm::vec3 rotationRates(0.0f);

//-------------------------------------------------------
// Utility Functions
//-------------------------------------------------------

// Get direction vectors from drone orientation
glm::vec3 getDroneFront() {
    return glm::rotate(droneQuat, glm::vec3(0.0f, 0.0f, -1.0f));
}

glm::vec3 getDroneUp() {
    return glm::rotate(droneQuat, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::vec3 getDroneRight() {
    return glm::rotate(droneQuat, glm::vec3(1.0f, 0.0f, 0.0f));
}

// Convert quaternion to euler angles for display
glm::vec3 quatToEuler(const glm::quat& quat) {
    glm::vec3 euler = glm::degrees(glm::eulerAngles(quat));

    // Normalize to -180 to 180 range
    if (euler.x > 180.0f) euler.x -= 360.0f;
    if (euler.y > 180.0f) euler.y -= 360.0f;
    if (euler.z > 180.0f) euler.z -= 360.0f;

    return euler;
}

// Apply exponential curve to control inputs for better precision
float applyExpo(float input, float expoFactor) {
    return input * (expoFactor * input * input + (1.0f - expoFactor));
}

// Calculate motor efficiency based on RPM and throttle
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

// Get battery type name as string
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

// Cycle battery type up (increase cell count)
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
    droneConfig.updateMinBatteryVoltage();
    battery.configureBattery(newType);
    initializeDrone();
}

// Cycle battery type down (decrease cell count)
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
    droneConfig.updateMinBatteryVoltage();
    battery.configureBattery(newType);
    initializeDrone();
}

// Cycle through display modes
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

// Initialize the drone with current configuration
void initializeDrone() {
    // 1. Configure motor positions based on frame size
    float motorOffset = droneConfig.frameSize / 2.0f;
    motors[0].position = glm::vec3(motorOffset, 0.0f, -motorOffset);  // Front-right (CCW)
    motors[1].position = glm::vec3(motorOffset, 0.0f, motorOffset);   // Rear-right (CW)
    motors[2].position = glm::vec3(-motorOffset, 0.0f, motorOffset);  // Rear-left (CCW)
    motors[3].position = glm::vec3(-motorOffset, 0.0f, -motorOffset); // Front-left (CW)

    // 2. Configure motor rotation directions for stability (alternating CW/CCW)
    motors[0].rotationDir = 1.0f;   // CCW
    motors[1].rotationDir = -1.0f;  // CW
    motors[2].rotationDir = 1.0f;   // CCW
    motors[3].rotationDir = -1.0f;  // CW

    // 3. Configure battery and update all dependent parameters
    battery.configureBattery(droneConfig.batteryType);
    battery.capacity = droneConfig.batteryCapacity;
    battery.charge = battery.capacity;  // Full charge on initialization
    battery.isDepleted = false;
    battery.currentVoltageSag = 0.0f;
    battery.baseBatteryVoltage = battery.maxVoltage;

    // 4. Initialize motor state
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Calculate maximum RPM based on battery and motor KV
        motors[i].maxRPM = battery.maxVoltage * droneConfig.motorKv;

        // Reset all motor states
        motors[i].targetThrottle = motors[i].currentThrottle = 0.0f;
        motors[i].currentRPM = motors[i].targetRPM = 0.0f;
        motors[i].thrust = 0.0f;
        motors[i].torque = 0.0f;
        motors[i].angularVelocity = 0.0f;
        motors[i].isSpinning = false;
        motors[i].currentDraw = 0.0f;
    }

    // 5. Position drone in 3D space and reset physics state
    dronePosition = glm::vec3(0.0f, 1.0f, 3.0f);  // Default starting position
    velocity = glm::vec3(0.0f);                   // No initial velocity
    droneQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // Level orientation
    angularVelocity = glm::vec3(0.0f);            // No initial rotation
    rotationRates = glm::vec3(0.0f);

    // 6. Reset control inputs
    controls.throttlePosition = 0.0f;
    input = {};  // Reset all input state

    // 7. Reset simulation timers
    simulationStartTime = std::chrono::steady_clock::now();
    lastFrameTime = 0.0f;
    deltaTime = 0.0f;
    previousTotalCurrentDraw = 0.0f;
    batteryUpdateTime = 0.0f;

    // 8. Always start in disarmed state for safety
    armed = false;

    // 9. Configure physical frame dimensions based on drone config
    droneFrame.propSizeInches = droneConfig.propSize;
    droneFrame.updateFromConfig(droneConfig);

    // 10. Set collision physics properties
    droneFrame.elasticity = 0.05f;  // Low bounce
    droneFrame.friction = 0.95f;    // High friction

    // 11. Log initialization
    float thrustToWeight = droneConfig.getThrustToWeightRatio();
    std::cout << "Drone initialized: "
        << droneFrame.getSizeString()
        << " | " << getBatteryTypeName(droneConfig.batteryType)
        << " | TWR: " << std::fixed << std::setprecision(1) << thrustToWeight << ":1"
        << std::endl;
}
//-------------------------------------------------------
// Physics Simulation
//-------------------------------------------------------

// Updates motor states based on target throttles and physics
void updateMotors(float deltaTime) {
    float totalCurrentDraw = 0.0f;

    // Calculate effective voltage with improved voltage sag model
    float batteryVoltageDrop = battery.sagFactor * battery.internalResistance * previousTotalCurrentDraw;

    // Store both the base voltage (without sag) and current voltage with sag
    battery.baseBatteryVoltage = battery.getVoltageFromCharge();
    battery.currentVoltageSag = batteryVoltageDrop;

    float effectiveVoltage = std::max(battery.baseBatteryVoltage - batteryVoltageDrop, battery.minVoltage * 0.9f);

    // Base electronics current - varies based on whether armed or not
    float baseElectronicsCurrent = armed ? 0.6f : 0.15f; // Higher when armed (FC, RX, camera, VTX)
    totalCurrentDraw += baseElectronicsCurrent;

    // Process each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].rotationDir = (i % 2 == 0) ? 1.0f : -1.0f;

        float targetThrottle = motors[i].targetThrottle;

        // Set minimum idle throttle when armed
        // This makes props spin at idle even with zero throttle input
        if (armed && targetThrottle < 0.05f && !battery.isDepleted &&
            battery.voltage >= droneConfig.minBatteryVoltageForFlight * 0.95f) {
            targetThrottle = 0.05f; // Minimum idle throttle when armed
        }

        // Calculate motor response rate - different for acceleration vs. deceleration
        float responseRate;
        if (targetThrottle > motors[i].currentThrottle) {
            responseRate = droneConfig.motorAcceleration * (0.8f + 0.2f * effectiveVoltage / battery.maxVoltage);
        }
        else {
            responseRate = droneConfig.motorDeceleration;
        }

        float throttleResponseFactor = 0.6f + 0.4f * motors[i].currentThrottle;
        responseRate *= throttleResponseFactor;

        // Update current throttle with ramping
        motors[i].currentThrottle = glm::mix(motors[i].currentThrottle, targetThrottle, deltaTime * responseRate);

        // Modified motor spinning logic to account for armed idle state
        bool shouldSpin = (armed && motors[i].currentThrottle > 0.01f) &&  // Allow very low throttle when armed
            !battery.isDepleted && battery.voltage >= droneConfig.minBatteryVoltageForFlight * 0.95f;

        // Motor state changes
        if (shouldSpin && !motors[i].isSpinning) {
            motors[i].isSpinning = true;
        }
        else if (!shouldSpin && motors[i].isSpinning) {
            motors[i].isSpinning = false;
        }

        // Update target RPM based on throttle and battery voltage
        if (motors[i].isSpinning) {
            float throttleCurve = powf(motors[i].currentThrottle, 1.1f); // Slight non-linearity
            motors[i].targetRPM = throttleCurve * effectiveVoltage * droneConfig.motorKv;

            // Ensure a minimum RPM when armed but idle
            if (armed && motors[i].targetRPM < 1000.0f) {
                motors[i].targetRPM = 1000.0f; // Minimum idle RPM
            }
        }
        else {
            motors[i].targetRPM = 0.0f;
        }

        // Battery depleted safety check
        if (battery.isDepleted || battery.voltage <= battery.minVoltage) {
            motors[i].targetRPM = 0.0f;
            motors[i].isSpinning = false;
        }

        // Torque calculation
        float maxTorque = 0.00012f * droneConfig.motorKv * effectiveVoltage * motors[i].currentThrottle;

        float torqueRPMFactor = 1.0f - (motors[i].currentRPM / (1.5f * motors[i].maxRPM + 0.001f));
        torqueRPMFactor = glm::clamp(torqueRPMFactor, 0.2f, 1.0f);
        motors[i].torque = maxTorque * torqueRPMFactor;

        // Angular acceleration calculation
        float currentRadS = motors[i].currentRPM * RPM_TO_RADS;
        float angularAcceleration = motors[i].torque / droneConfig.motorInertia;

        // Rotational friction/drag
        float rotationalFriction = 0.000008f * motors[i].currentRPM;
        angularAcceleration -= rotationalFriction * currentRadS / droneConfig.motorInertia;

        // Update angular velocity
        motors[i].angularVelocity += angularAcceleration * deltaTime;
        motors[i].angularVelocity = glm::max(0.0f, motors[i].angularVelocity);
        motors[i].currentRPM = motors[i].angularVelocity / RPM_TO_RADS;

        // Thrust calculation
        float rps = motors[i].currentRPM / 60.0f;
        float propDiameterM = droneConfig.propSize * INCH_TO_METER;
        float rpm_factor = powf(rps, 2.3f);

        motors[i].thrust = droneConfig.thrustCoefficient * aero.airDensity * rpm_factor *
            powf(propDiameterM, 4.0f) * droneConfig.propEfficiency;

        // Apply voltage efficiency factor
        float voltageEfficiencyFactor = glm::clamp(effectiveVoltage / battery.nominalVoltage, 0.8f, 1.8f);
        motors[i].thrust *= voltageEfficiencyFactor;

        // Special case for low RPM at high throttle (motor startup boost)
        if (motors[i].currentRPM < motors[i].maxRPM * 0.3f && motors[i].targetThrottle > 0.15f) {
            motors[i].thrust *= 1.2f;
        }

        // Adjust thrust for forward speed (prop stall effect)
        float forwardSpeed = glm::length(velocity);
        if (forwardSpeed > 8.0f) {
            float stallFactor = 1.0f - glm::clamp((forwardSpeed - 8.0f) / 40.0f, 0.0f, 0.25f);
            motors[i].thrust *= stallFactor;
        }

        // Current calculation for battery consumption
        if (motors[i].isSpinning) {
            // Fixed current component (ESC & motor at idle)
            float idleCurrent = 0.2f; // Base current draw when motor spins at minimum

            // Current based on throttle position - use a quadratic model for better realism
            // Higher cell count = more power
            float throttleCurrent = powf(motors[i].currentThrottle, 2.0f) * (battery.cellCount * 3.0f);

            // Startup current surge (when motor is accelerating rapidly)
            float accelerationCurrent = 0.0f;
            if (motors[i].targetThrottle > motors[i].currentThrottle + 0.02f) {
                // Higher surge when starting from stop
                float startupFactor = motors[i].currentRPM < 5000.0f ? 2.0f : 1.0f;
                accelerationCurrent = (motors[i].targetThrottle - motors[i].currentThrottle) * 5.0f * startupFactor;
            }

            // Combine current components
            motors[i].currentDraw = idleCurrent + throttleCurrent + accelerationCurrent;

            // Add voltage sag effect (more current at lower voltage)
            if (effectiveVoltage < battery.nominalVoltage) {
                float voltageFactor = battery.nominalVoltage / effectiveVoltage;
                motors[i].currentDraw *= glm::min(voltageFactor * 1.1f, 1.5f);
            }

            // Cap maximum current to realistic levels
            float maxMotorCurrentDraw = battery.cellCount * 8.0f; // ~8A per cell is realistic
            motors[i].currentDraw = glm::min(motors[i].currentDraw, maxMotorCurrentDraw);
        }
        else {
            motors[i].currentDraw = 0.0f;
        }

        totalCurrentDraw += motors[i].currentDraw;
    }

    // Smooth current change rate
    float currentChangeRate = totalCurrentDraw > previousTotalCurrentDraw ? 8.0f : 15.0f;
    float smoothedTotalCurrentDraw = glm::mix(previousTotalCurrentDraw, totalCurrentDraw,
        glm::min(1.0f, deltaTime * currentChangeRate));

    // BATTERY CHARGE CALCULATION - CRITICAL SECTION
    // Convert time from seconds to hours
    float hoursElapsed = deltaTime / 3600.0f;

    // Calculate battery drain in mAh
    float baseDrainFactor = 3.5f; // Base multiplication factor for realistic drain rate

    // Dynamic factor based on battery capacity and style of flight
    float aggressivenessFactor = 1.0f;
    static float prevThrottle = 0.0f;
    static float maxThrottleChange = 0.0f;

    // Calculate throttle change rate
    float throttleChange = std::abs(controls.throttlePosition - prevThrottle) / deltaTime;
    maxThrottleChange = glm::max(maxThrottleChange * 0.95f, throttleChange);

    if (maxThrottleChange > 1.0f) {
        aggressivenessFactor = 1.0f + glm::min(maxThrottleChange * 0.1f, 0.5f);
    }
    prevThrottle = controls.throttlePosition;

    // Apply battery efficiency loss at lower charge levels
    float chargeLevel = battery.charge / battery.capacity;
    float efficiencyFactor = chargeLevel > 0.3f ? 1.0f : (1.0f + (0.3f - chargeLevel) * 2.0f);

    // Calculate final drain
    float mAhDrain = smoothedTotalCurrentDraw * hoursElapsed * 1000.0f * baseDrainFactor *
        aggressivenessFactor * efficiencyFactor;

    // Apply the drain
    battery.charge -= mAhDrain;
    battery.charge = glm::max(0.0f, battery.charge);

    // Store load level for recovery calculation
    battery.lastLoadLevel = smoothedTotalCurrentDraw;

    // Set battery to depleted if charge is low
    if (battery.charge < battery.capacity * 0.02f) {
        battery.isDepleted = true;
    }

    // Store current draw for next frame
    previousTotalCurrentDraw = smoothedTotalCurrentDraw;
    batteryUpdateTime += deltaTime;

    // Keep minimal thrust for armed idle state
    if (armed && controls.throttlePosition < 0.01f) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].thrust *= 0.05f;  // Keep minimal thrust for idle
        }
    }
}

// Sets throttle values for each motor based on throttle position and rotation rates
void setMotorThrottles(float mainThrottle, const glm::vec3& rotationRates) {
    // Check if battery is depleted or if not armed - in both cases, no throttle
    if (!armed || battery.isDepleted || battery.voltage <= droneConfig.minBatteryVoltageForFlight * 0.9f) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].targetThrottle = 0.0f;
        }

        // If battery is depleted, disarm the drone for safety
        if (battery.isDepleted || battery.voltage <= droneConfig.minBatteryVoltageForFlight * 0.9f) {
            if (armed) {
                armed = false;
                std::cout << "Drone auto-disarmed due to depleted battery!" << std::endl;
            }
        }

        return;
    }

    float pitchEffect = rotationRates.x * 0.2f;
    float rollEffect = rotationRates.z * 0.2f;
    float yawEffect = rotationRates.y * 0.15f;

    motors[0].targetThrottle = mainThrottle - pitchEffect - rollEffect + yawEffect; // Front-right
    motors[1].targetThrottle = mainThrottle + pitchEffect - rollEffect - yawEffect; // Rear-right
    motors[2].targetThrottle = mainThrottle + pitchEffect + rollEffect + yawEffect; // Rear-left
    motors[3].targetThrottle = mainThrottle - pitchEffect + rollEffect - yawEffect; // Front-left

    // Clamp throttle values to valid range
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].targetThrottle = glm::clamp(motors[i].targetThrottle, 0.0f, MAX_THROTTLE);
    }

    // Apply a boost when throttle is increasing rapidly
    static float prevThrottle = 0.0f;
    if (mainThrottle > prevThrottle + 0.02f) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].targetThrottle = glm::min(motors[i].targetThrottle * 1.1f, MAX_THROTTLE);
        }
    }
    prevThrottle = mainThrottle;
}

// Update contact points with ground
void updateGroundContacts() {
    groundContact.reset();
    groundContact.updateBoundingBox(dronePosition, droneFrame);

    float groundDetectionThreshold = 0.03f; // Small threshold to detect ground

    // Transform all contact points from local to world space
    for (int i = 0; i < DroneFrameSize::NUM_CONTACT_POINTS; i++) {
        // Transform point from local to world space
        groundContact.worldPoints[i] = dronePosition + glm::rotate(droneQuat, droneFrame.contactPoints[i]);

        // Distance to ground - simple Y coordinate
        groundContact.groundDistances[i] = groundContact.worldPoints[i].y;

        // Check if contact point is touching or penetrating ground
        if (groundContact.groundDistances[i] <= groundDetectionThreshold) {
            groundContact.inContact[i] = true;
            groundContact.totalContactsCount++;

            // Calculate penetration depth
            groundContact.penetrationDepths[i] = groundDetectionThreshold - groundContact.groundDistances[i];

            // Calculate impact force (proportional to penetration and point mass)
            groundContact.impactForces[i] = groundContact.penetrationDepths[i] * 200.0f *
                droneFrame.contactPointMass[i] * droneConfig.mass;

            // Always use up vector as impact normal for flat ground
            groundContact.impactNormals[i] = glm::vec3(0.0f, 1.0f, 0.0f);

            // Track maximum penetration depth for collision response
            if (groundContact.penetrationDepths[i] > groundContact.maxPenetrationDepth) {
                groundContact.maxPenetrationDepth = groundContact.penetrationDepths[i];
            }
        }
    }

    // Compute side distributions for rotation response
    if (groundContact.totalContactsCount > 0) {
        groundContact.isGrounded = true;

        // Calculate load distribution across contact points
        float rightContacts = 0.0f, leftContacts = 0.0f;
        float frontContacts = 0.0f, backContacts = 0.0f;
        float rightForce = 0.0f, leftForce = 0.0f;
        float frontForce = 0.0f, backForce = 0.0f;

        for (int i = 0; i < DroneFrameSize::NUM_CONTACT_POINTS; i++) {
            if (groundContact.inContact[i]) {
                float contactForce = groundContact.impactForces[i];

                // Right side (0,1,4,5)
                if (i == 0 || i == 1 || i == 4 || i == 5) {
                    rightContacts += 1.0f;
                    rightForce += contactForce;
                }
                // Left side (2,3,6,7)
                else if (i == 2 || i == 3 || i == 6 || i == 7) {
                    leftContacts += 1.0f;
                    leftForce += contactForce;
                }

                // Front side (0,3,4,7)
                if (i == 0 || i == 3 || i == 4 || i == 7) {
                    frontContacts += 1.0f;
                    frontForce += contactForce;
                }
                // Back side (1,2,5,6)
                else if (i == 1 || i == 2 || i == 5 || i == 6) {
                    backContacts += 1.0f;
                    backForce += contactForce;
                }
            }
        }

        // Calculate balance ratios (-1 to 1)
        float totalSideContacts = rightContacts + leftContacts;
        float totalFBContacts = frontContacts + backContacts;

        if (totalSideContacts > 0) {
            groundContact.leftRightBalance = (rightContacts - leftContacts) / totalSideContacts;
        }

        if (totalFBContacts > 0) {
            groundContact.frontBackBalance = (backContacts - frontContacts) / totalFBContacts;
        }

        // Calculate stability - drone is stable if it has at least 3 contact points or center + 1 point
        groundContact.isStable = (groundContact.totalContactsCount >= 3) ||
            (groundContact.inContact[8] && groundContact.totalContactsCount >= 2);
    }

    // If drone is close enough to ground, always consider it grounded
    // This prevents "hovering" just above ground due to numerical precision issues
    if (dronePosition.y < (droneFrame.frameHeightMeters * 0.5f + 0.03f)) {
        groundContact.isGrounded = true;
    }
}

// Check if drone is in contact with ground
bool isDroneOnGround() {
    updateGroundContacts();
    return groundContact.isGrounded;
}

// Handle collision response when drone hits ground
void handleGroundCollision() {
    if (!groundContact.isGrounded) {
        return;
    }

    // Physical properties for collision response
    float frameElasticity = armed ? droneFrame.elasticity * 0.3f : droneFrame.elasticity * 0.1f;
    float frameFriction = armed ? droneFrame.friction : droneFrame.friction * 1.2f;

    // 1. Resolve penetration - move drone out of the ground to prevent sinking
    float minHeight = droneFrame.frameHeightMeters * 0.5f + 0.01f;
    if (dronePosition.y < minHeight) {
        dronePosition.y = minHeight;
    }

    // 2. Calculate collision impulse
    float impactVelocity = -velocity.y; // Negative since we're measuring impact toward ground

    if (impactVelocity > 0.01f) {
        // Apply elastic response with damping - more bounce when disarmed for realism
        velocity.y = impactVelocity * frameElasticity;

        // Rotational response based on which side hit first
        if (groundContact.totalContactsCount > 0) {
            // Apply torque based on contact distribution
            glm::vec3 contactTorque(0.0f);

            // More pronounced impact effects when disarmed
            float impactScale = armed ? 1.0f : 1.5f;

            // Pitch torque (front/back balance)
            contactTorque.x = -groundContact.frontBackBalance * impactVelocity * 0.1f * impactScale;

            // Roll torque (left/right balance)
            contactTorque.z = groundContact.leftRightBalance * impactVelocity * 0.1f * impactScale;

            // Add impulse torque to angular velocity
            angularVelocity += contactTorque;

            // Dampen angular velocity proportional to impact
            float impactDamping = glm::min(impactVelocity * 0.1f, 0.9f);
            angularVelocity *= (1.0f - impactDamping);
        }
    }
    else {
        // For small impacts or resting contact, kill vertical velocity
        velocity.y = 0.0f;
    }

    // Always apply ground friction to horizontal velocity when on ground
    float horizontalSpeed = glm::length(glm::vec2(velocity.x, velocity.z));

    if (horizontalSpeed > 0.01f) {
        // More gradual friction application for better sliding motion
        float frictionFactor = frameFriction * (0.5f + 0.5f / (1.0f + horizontalSpeed));
        frictionFactor = glm::clamp(frictionFactor, 0.0f, 0.95f);

        // Apply friction over time - more gradual to allow for sliding
        float frictionStrength = armed ? 10.0f : 5.0f; // Reduced for disarmed state
        velocity.x *= (1.0f - frictionFactor * deltaTime * frictionStrength);
        velocity.z *= (1.0f - frictionFactor * deltaTime * frictionStrength);
    }
    else {
        // Only kill very small velocities to prevent jittering
        if (horizontalSpeed < 0.001f) {
            velocity.x = 0.0f;
            velocity.z = 0.0f;
        }
    }

    // Apply orientation stabilization when drone is on ground
    glm::vec3 droneUp = getDroneUp();
    glm::vec3 worldUp(0.0f, 1.0f, 0.0f);

    // Calculate rotation axis to level the drone
    glm::vec3 rotAxis = glm::cross(droneUp, worldUp);
    float rotAxisLength = glm::length(rotAxis);

    // Only apply rotation correction if the drone is tilted
    if (rotAxisLength > 0.001f) {
        rotAxis = glm::normalize(rotAxis);
        float alignmentAngle = std::acos(glm::clamp(glm::dot(droneUp, worldUp), -1.0f, 1.0f));

        // Different stabilization rates based on arming state
        // Much slower stabilization when disarmed for more natural tumbling
        float stabilizationRate = armed ? 2.0f : 1.0f;
        float maxCorrection = armed ? 0.1f : 0.1f;

        // Apply gradual orientation correction
        float correctionStrength = std::min(stabilizationRate * deltaTime, maxCorrection);
        glm::quat alignmentRotation = glm::angleAxis(alignmentAngle * correctionStrength, rotAxis);
        droneQuat = glm::normalize(alignmentRotation * droneQuat);

        // Add additional damping to angular velocity
        angularVelocity *= armed ? 0.95f : 0.98f;
    }

    // Ensure the drone can't fall through the ground
    if (dronePosition.y < minHeight) {
        dronePosition.y = minHeight;
        velocity.y = std::max(0.0f, velocity.y); // Prevent negative vertical velocity
    }
}

// Update input state based on keyboard
void updateInputState(GLFWwindow* window) {
    input.throttleUp = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    input.throttleDown = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;

    input.pitchForward = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    input.pitchBackward = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    input.rollLeft = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    input.rollRight = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    input.yawLeft = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    input.yawRight = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;

    input.batteryTypeUp = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
    input.batteryTypeDown = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;

    input.armToggle = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;

    input.keyPressed = input.throttleUp || input.throttleDown ||
        input.pitchForward || input.pitchBackward ||
        input.rollLeft || input.rollRight ||
        input.yawLeft || input.yawRight ||
        input.batteryTypeUp || input.batteryTypeDown ||
        input.armToggle;
}

// Main input processing and physics update
void processInput(GLFWwindow* window) {
    float currentTime = static_cast<float>(glfwGetTime());
    deltaTime = currentTime - lastFrameTime;
    lastFrameTime = currentTime;

    // Cap deltaTime to avoid large jumps in physics when framerate drops or debugging
    if (deltaTime > 0.05f) deltaTime = 0.05f;

    // Handle keyboard inputs
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

    // Camera angle controls
    if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        cameraAngle += CAMERA_ANGLE_INCREMENT;
        if (cameraAngle > 45.0f) cameraAngle = 45.0f; // Maximum upward angle
        lastKeyPressTime = currentTime;
    }

    if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        cameraAngle -= CAMERA_ANGLE_INCREMENT;
        if (cameraAngle < -45.0f) cameraAngle = -45.0f; // Maximum downward angle
        lastKeyPressTime = currentTime;
    }

    // Arm/Disarm toggle
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS && (currentTime - lastKeyPressTime) > keyDebounceTime) {
        // Update ground contact check first
        bool onGround = isDroneOnGround();

        if (!armed) {
            // Can only arm when on ground
            if (onGround) {
                armed = true;
                std::cout << "Drone armed! Press Q to disarm." << std::endl;
                lastKeyPressTime = currentTime;
            }
            else {
                std::cout << "Can't arm: drone must be on ground." << std::endl;
            }
        }
        else {
            // Can always disarm
            armed = false;
            std::cout << "Drone disarmed!" << std::endl;

            // Reset control inputs but KEEP MOMENTUM
            controls.throttlePosition = 0.0f;
            input.pitchInput = 0.0f;
            input.rollInput = 0.0f;
            input.yawInput = 0.0f;
            rotationRates = glm::vec3(0.0f);

            // Stop all motors but don't change velocity
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i].targetThrottle = 0.0f;
                motors[i].currentThrottle = 0.0f;
                motors[i].isSpinning = false;
                motors[i].thrust = 0.0f;
            }

            lastKeyPressTime = currentTime;
        }
    }

    updateInputState(window);

    // Only process throttle input when armed
    if (armed) {
        if (input.throttleUp && !input.throttleDown) {
            controls.throttlePosition += controls.throttleIncrement * deltaTime;
        }
        else if (input.throttleDown && !input.throttleUp) {
            controls.throttlePosition -= controls.throttleIncrement * deltaTime;
        }
        else if (controls.throttleReturnToZero) {
            controls.throttlePosition = glm::mix(controls.throttlePosition, 0.0f, deltaTime * controls.throttleResponseRate);
        }
    }
    else {
        // When disarmed, no throttle
        controls.throttlePosition = 0.0f;
    }

    controls.throttlePosition = glm::clamp(controls.throttlePosition, 0.0f, 1.0f);
    float mainThrottle = controls.throttlePosition * MAX_THROTTLE;

    glm::vec3 rotationInput(0.0f);

    // Only process control inputs when armed
    if (armed) {
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
    }
    else {
        // When disarmed, reset all inputs
        input.pitchInput = 0.0f;
        input.rollInput = 0.0f;
        input.yawInput = 0.0f;
    }

    // Apply expo curve to control inputs
    float expoFactor = 0.6f;
    rotationInput.x = applyExpo(input.pitchInput, expoFactor);
    rotationInput.z = applyExpo(input.rollInput, expoFactor);
    rotationInput.y = applyExpo(input.yawInput, expoFactor);

    rotationInput *= controls.rotationSensitivity;

    // Adjust response based on battery voltage
    float voltageRatio = glm::clamp((battery.voltage - battery.minVoltage) /
        (battery.maxVoltage - battery.minVoltage), 0.6f, 1.0f);

    float responseRate = 5.0f * voltageRatio;

    // Only update rotation rates when armed
    if (armed) {
        rotationRates = glm::mix(rotationRates, rotationInput * controls.maxRotationRate * voltageRatio, deltaTime * responseRate);
    }
    else {
        // When disarmed, no rotation input
        rotationRates = glm::vec3(0.0f);
    }

    // Apply rotational drag
    rotationRates *= (1.0f - ROTATIONAL_DRAG * deltaTime);

    // Set motor throttles based on control inputs
    setMotorThrottles(mainThrottle, rotationRates);
    updateMotors(deltaTime);

    // Convert body rates to world space angular velocity
    glm::mat3 rotMatrix = glm::mat3_cast(droneQuat);
    glm::vec3 bodyAngularVelocity = glm::vec3(
        rotationRates.x,
        rotationRates.y,
        rotationRates.z
    );

    angularVelocity = rotMatrix * bodyAngularVelocity;

    // Special handling for disarmed drone on ground
    if (isDroneOnGround() && !armed) {
        velocity *= 0.98f; // Less damping to preserve momentum

        // Apply stabilizing torque to ensure flat orientation
        glm::vec3 droneUp = getDroneUp();
        glm::vec3 worldUp = glm::vec3(0.0f, 1.0f, 0.0f);

        glm::vec3 rotAxis = glm::cross(droneUp, worldUp);
        if (glm::length(rotAxis) > 0.001f) {
            rotAxis = glm::normalize(rotAxis);
            float alignmentAngle = std::acos(glm::clamp(glm::dot(droneUp, worldUp), -1.0f, 1.0f));

            // Create stabilizing rotation - less aggressive to keep some angular momentum
            float stabilizationStrength = 1.5f * deltaTime;
            angularVelocity += rotAxis * alignmentAngle * stabilizationStrength;
            angularVelocity *= 0.95f; // Less damping to preserve angular momentum
        }

        // Apply angular velocity with damping
        float angularVelocityMagnitude = glm::length(angularVelocity);
        if (angularVelocityMagnitude > 0.001f) {
            float angle = angularVelocityMagnitude * deltaTime;
            glm::vec3 axis = angularVelocity / angularVelocityMagnitude;
            glm::quat deltaQuat = glm::angleAxis(angle, axis);
            droneQuat = glm::normalize(deltaQuat * droneQuat);
        }
    }
    else {
        // Normal movement and rotation calculation
        float angularVelocityMagnitude = glm::length(angularVelocity);
        if (angularVelocityMagnitude > 0.001f) {
            float angle = angularVelocityMagnitude * deltaTime;
            glm::vec3 axis = angularVelocity / angularVelocityMagnitude;
            glm::quat deltaQuat = glm::angleAxis(angle, axis);
            droneQuat = glm::normalize(deltaQuat * droneQuat);
        }
    }

    // Get drone orientation vectors
    glm::vec3 droneUp = getDroneUp();
    glm::vec3 droneFront = getDroneFront();
    glm::vec3 droneRight = getDroneRight();

    // Calculate forces
    glm::vec3 thrustForce(0.0f);
    for (int i = 0; i < NUM_MOTORS; i++) {
        thrustForce += droneUp * motors[i].thrust;
    }

    glm::vec3 gravityForceVec = glm::vec3(0.0f, -GRAVITY * droneConfig.mass * 0.93f, 0.0f);

    // Apply ground effect (increased thrust near ground)
    float groundEffectMultiplier = 1.0f;
    if (dronePosition.y < aero.groundEffectHeight * 1.2f) {
        groundEffectMultiplier = 1.0f + aero.groundEffectStrength * 3.0f *
            (1.0f - dronePosition.y / (aero.groundEffectHeight * 1.2f));
        thrustForce.y *= groundEffectMultiplier;
    }

    // Calculate prop wash (turbulence)
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

    // Apply drag even when disarmed for realistic momentum loss
    glm::vec3 dragForce(0.0f);
    if (glm::length(velocity) > 0.001f) {
        float frontalArea = droneConfig.frameSize * droneConfig.frameSize * 0.5f;
        float dynamicPressure = 0.5f * aero.airDensity * glm::length(velocity) * glm::length(velocity);

        // Increase drag slightly when disarmed (tumbling drone has more drag)
        float dragMultiplier = armed ? 1.0f : 1.5f;

        float forwardDrag = glm::abs(glm::dot(velocity, droneFront)) * dynamicPressure *
            frontalArea * aero.dragCoefficients.z * dragMultiplier;
        float sideDrag = glm::abs(glm::dot(velocity, droneRight)) * dynamicPressure *
            frontalArea * aero.dragCoefficients.x * dragMultiplier;
        float verticalDrag = glm::abs(glm::dot(velocity, droneUp)) * dynamicPressure *
            frontalArea * aero.dragCoefficients.y * dragMultiplier;

        dragForce = -glm::normalize(velocity) * (forwardDrag + sideDrag + verticalDrag);
    }

    // Calculate gyroscopic torque from spinning motors
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

    // Calculate total force with or without thrust based on armed state
    glm::vec3 totalForce = gravityForceVec + dragForce;
    if (armed) {
        totalForce += thrustForce + propWash;
    }
    else {
        // If disarmed but in air, add some random torque to simulate real drone tumbling
        if (!isDroneOnGround()) {
            float tumbleStrength = 0.2f;
            glm::vec3 randomTorque(
                ((float)rand() / RAND_MAX * 2.0f - 1.0f) * tumbleStrength,
                ((float)rand() / RAND_MAX * 2.0f - 1.0f) * tumbleStrength,
                ((float)rand() / RAND_MAX * 2.0f - 1.0f) * tumbleStrength
            );
            angularVelocity += randomTorque * deltaTime;
        }
    }

    // Apply forces to update velocity and position
    glm::vec3 acceleration = totalForce / droneConfig.mass;
    velocity += acceleration * deltaTime;
    dronePosition += velocity * deltaTime;

    // Handle ground collisions
    updateGroundContacts();
    if (groundContact.isGrounded) {
        handleGroundCollision();
    }

    // Apply a constant minimum height constraint based on the drone's frame size
    float absoluteMinHeight = droneFrame.frameHeightMeters * 0.5f;
    if (dronePosition.y < absoluteMinHeight) {
        dronePosition.y = absoluteMinHeight;
        velocity.y = 0.0f;  // Kill vertical velocity when hitting absolute floor
    }
}

//-------------------------------------------------------
// OpenGL and Rendering
//-------------------------------------------------------

// Window resize callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    currentWidth = width;
    currentHeight = height;
}

// Get OpenGL version information
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

// Compile shader
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

// Setup shader program
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

// Setup floor and grid buffers
void setupFloorBuffers() {
    // Floor vertices - 1000x1000 meter ground
    float floorVertices[] = {
        -500.0f, 0.0f, -500.0f, 0.3f, 0.3f, 0.3f,
         500.0f, 0.0f, -500.0f, 0.3f, 0.3f, 0.3f,
         500.0f, 0.0f,  500.0f, 0.3f, 0.3f, 0.3f,
        -500.0f, 0.0f,  500.0f, 0.3f, 0.3f, 0.3f
    };

    // Create floor buffer
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

    // Create grid lines
    std::vector<float> gridLines;
    float gridColor[] = { 0.5f, 0.5f, 0.5f };
    float gridSpacing = 1.0f;  // 1 meter grid

    for (float i = -500.0f; i <= 500.0f; i += gridSpacing) {
        // X-direction lines
        gridLines.push_back(i); gridLines.push_back(0.01f); gridLines.push_back(-500.0f);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);

        gridLines.push_back(i); gridLines.push_back(0.01f); gridLines.push_back(500.0f);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);

        // Z-direction lines
        gridLines.push_back(-500.0f); gridLines.push_back(0.01f); gridLines.push_back(i);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);

        gridLines.push_back(500.0f); gridLines.push_back(0.01f); gridLines.push_back(i);
        gridLines.push_back(gridColor[0]); gridLines.push_back(gridColor[1]); gridLines.push_back(gridColor[2]);
    }

    // Create grid buffer
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

    // Create coordinate axes
    float axesVertices[] = {
        -500.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // X-axis (red)
         500.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, -500.0f, 0.0f, 0.0f, 1.0f,  // Z-axis (blue)
         0.0f, 0.0f,  500.0f, 0.0f, 0.0f, 1.0f
    };

    // Create axes buffer
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

// Setup skybox buffer
void setupSkyboxBuffers() {
    float farDistance = 1000.0f;  // Match with floor size
    std::vector<float> skyboxVertices;
    glm::vec3 skyColor(0.95f, 0.65f, 0.35f);  // Warm orange
    float radius = farDistance;

    // Create hemisphere background vertices
    // Bottom left
    skyboxVertices.push_back(-radius);
    skyboxVertices.push_back(0.0f);
    skyboxVertices.push_back(-radius);
    skyboxVertices.push_back(skyColor.r);
    skyboxVertices.push_back(skyColor.g);
    skyboxVertices.push_back(skyColor.b);

    // Bottom right
    skyboxVertices.push_back(radius);
    skyboxVertices.push_back(0.0f);
    skyboxVertices.push_back(-radius);
    skyboxVertices.push_back(skyColor.r);
    skyboxVertices.push_back(skyColor.g);
    skyboxVertices.push_back(skyColor.b);

    // Top middle
    skyboxVertices.push_back(0.0f);
    skyboxVertices.push_back(radius);
    skyboxVertices.push_back(-radius);
    skyboxVertices.push_back(skyColor.r);
    skyboxVertices.push_back(skyColor.g);
    skyboxVertices.push_back(skyColor.b);

    // Top left
    skyboxVertices.push_back(-radius);
    skyboxVertices.push_back(radius);
    skyboxVertices.push_back(-radius);
    skyboxVertices.push_back(skyColor.r);
    skyboxVertices.push_back(skyColor.g);
    skyboxVertices.push_back(skyColor.b);

    // Top right
    skyboxVertices.push_back(radius);
    skyboxVertices.push_back(radius);
    skyboxVertices.push_back(-radius);
    skyboxVertices.push_back(skyColor.r);
    skyboxVertices.push_back(skyColor.g);
    skyboxVertices.push_back(skyColor.b);

    // Create sun vertices
    float sunRadius = radius * 0.08f;
    float sunHeight = radius * 0.2f;
    glm::vec3 sunPosition(0.0f, sunHeight, -radius * 0.9f);
    glm::vec3 sunColor(1.0f, 0.9f, 0.7f);
    glm::vec3 sunGlowColor(0.98f, 0.8f, 0.3f);

    // Create sun as a series of triangles
    int sunSegments = 16;
    for (int i = 0; i < sunSegments; i++) {
        float angle1 = float(i) / float(sunSegments) * 2.0f * PI;
        float angle2 = float((i + 1) % sunSegments) / float(sunSegments) * 2.0f * PI;

        // Sun center
        skyboxVertices.push_back(sunPosition.x);
        skyboxVertices.push_back(sunPosition.y);
        skyboxVertices.push_back(sunPosition.z);
        skyboxVertices.push_back(sunColor.r);
        skyboxVertices.push_back(sunColor.g);
        skyboxVertices.push_back(sunColor.b);

        // First point on edge
        skyboxVertices.push_back(sunPosition.x + sunRadius * std::cos(angle1));
        skyboxVertices.push_back(sunPosition.y + sunRadius * std::sin(angle1));
        skyboxVertices.push_back(sunPosition.z);
        skyboxVertices.push_back(sunGlowColor.r);
        skyboxVertices.push_back(sunGlowColor.g);
        skyboxVertices.push_back(sunGlowColor.b);

        // Second point on edge
        skyboxVertices.push_back(sunPosition.x + sunRadius * std::cos(angle2));
        skyboxVertices.push_back(sunPosition.y + sunRadius * std::sin(angle2));
        skyboxVertices.push_back(sunPosition.z);
        skyboxVertices.push_back(sunGlowColor.r);
        skyboxVertices.push_back(sunGlowColor.g);
        skyboxVertices.push_back(sunGlowColor.b);
    }

    // Create skybox buffer
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);

    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, skyboxVertices.size() * sizeof(float), skyboxVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

// Draw floor and grid
void drawFloor(const glm::mat4& projection, const glm::mat4& view) {
    glUseProgram(shaderProgram);

    unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
    unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    // Draw floor
    glBindVertexArray(floorVAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    // Draw grid
    glBindVertexArray(gridVAO);
    int gridLineCount = 1001; // 1000 meters + 1 (-500 to +500 with 1m spacing)
    int gridVertexCount = gridLineCount * 2 * 2; // Lines in both directions
    glDrawArrays(GL_LINES, 0, gridVertexCount);

    // Draw coordinate axes
    glLineWidth(2.0f);
    glBindVertexArray(axesVAO);
    glDrawArrays(GL_LINES, 0, 4);
    glLineWidth(1.0f);

    glBindVertexArray(0);
}

// Draw skybox
void drawSkybox(const glm::mat4& projection, const glm::mat4& view) {
    glUseProgram(shaderProgram);

    // Use depth equal for skybox
    glDepthFunc(GL_LEQUAL);

    unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
    unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");

    // Use rotation-only view matrix for skybox
    glm::mat4 skyView = glm::mat4(glm::mat3(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(skyView));

    glBindVertexArray(skyboxVAO);

    // Draw sky background
    glDrawArrays(GL_TRIANGLE_FAN, 0, 5);

    // Draw sun (triangles)
    int backgroundVertexCount = 5;
    int sunOffset = backgroundVertexCount;
    int sunVertexCount = 16 * 3; // sunSegments * 3 vertices per triangle
    glDrawArrays(GL_TRIANGLES, sunOffset, sunVertexCount);

    // Restore depth function
    glDepthFunc(GL_LESS);

    glBindVertexArray(0);
}
//-------------------------------------------------------
// UI Rendering
//-------------------------------------------------------

// Update window title with drone and simulation info
void updateWindowTitle(GLFWwindow* window, float fps) {
    std::stringstream title;
    glm::vec3 eulerAngles = quatToEuler(droneQuat);

    float speedMS = glm::length(velocity);
    float speedKMH = speedMS * MS_TO_KMH;

    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsedSeconds = currentTime - simulationStartTime;
    int elapsedMinutes = static_cast<int>(elapsedSeconds.count()) / 60;
    int elapsedSecondsRemainder = static_cast<int>(elapsedSeconds.count()) % 60;

    float batteryPercentage = (battery.charge / battery.capacity) * 100.0f;

    // If battery is depleted, show a more prominent warning
    if (battery.isDepleted) {
        title << "!!! BATTERY DEPLETED !!! | ";
    }

    title << "FPV Drone Sim | ";
    title << (armed ? "ARMED | " : "DISARMED | ");
    title << "FPS: " << std::fixed << std::setprecision(1) << fps << " | ";
    title << getBatteryTypeName(droneConfig.batteryType) << " | ";
    title << droneFrame.frameSizeInches << "\" Frame | ";
    title << "FPV Cam: " << std::setprecision(0) << std::showpos << cameraAngle << "° | ";

    if (showSimulationInfo) {
        title << "Speed: " << std::setprecision(1) << speedKMH << " km/h | "
            << "Alt: " << std::setprecision(1) << dronePosition.y << "m | ";
    }

    if (showControlInfo) {
        title << "Throttle: " << std::setprecision(2) << controls.throttlePosition << " | ";
    }

    if (showBatteryInfo) {
        // Show actual battery charge percentage
        title << "Batt: " << std::setprecision(1) << batteryPercentage << "% | ";

        // Show voltage with an indication when it's under load
        float currentVoltage = battery.baseBatteryVoltage - battery.currentVoltageSag;
        title << std::setprecision(1) << currentVoltage << "V";

        // Show base voltage when under load
        if (battery.currentVoltageSag > 0.2f) {
            title << " (" << std::setprecision(1) << battery.baseBatteryVoltage << "V no load)";
        }

        // Warning system based on actual battery charge, not temporary voltage sag
        if (battery.isDepleted) {
            title << " [EMPTY]";
        }
        else if (batteryPercentage < 15.0f) {
            title << " [LOW!]";     // Very low capacity
        }
        else if (batteryPercentage < 30.0f) {
            title << " [Low]";      // Low capacity warning
        }

        // Show current draw when armed
        if (armed && showMotorInfo) {
            float totalAmps = 0.0f;
            for (int i = 0; i < NUM_MOTORS; i++) {
                totalAmps += motors[i].currentDraw;
            }
            totalAmps += 0.6f;  // Add base electronics current
            title << " | " << std::setprecision(1) << totalAmps << "A";
        }

        title << " | Flight: " << elapsedMinutes << ":" << std::setw(2) << std::setfill('0')
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

// Function to draw virtual joystick controls
void renderJoystickControls() {
    ImGuiIO& io = ImGui::GetIO();

    // Position window at bottom-center
    constexpr ImVec2 windowSize{ 300.0f, 160.0f };
    ImVec2 windowPos{
        (io.DisplaySize.x - windowSize.x) * 0.5f,
        io.DisplaySize.y - windowSize.y - 10.0f
    };
    ImGui::SetNextWindowPos(windowPos);
    ImGui::SetNextWindowSize(windowSize);

    // Create transparent, fixed window
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoSavedSettings;
    ImGui::Begin("Joystick Controls", nullptr, flags);

    // Update joystick state from input
    joystickState.leftStick = { 0.5f + input.yawInput * 0.5f,
                               0.5f + (controls.throttlePosition - 0.5f) };
    joystickState.rightStick = { 0.5f - input.rollInput * 0.5f,
                                0.5f - input.pitchInput * 0.5f };

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Base dimensions
    constexpr float size = 100.0f;
    constexpr float rad = size * 0.5f;
    constexpr float stick = 8.0f;
    float gap = (windowSize.x - size * 2.0f) / 3.0f;

    // Calculate centers
    ImVec2 centers[2] = {
        { windowPos.x + gap + rad, windowPos.y + windowSize.y * 0.5f },
        { windowPos.x + gap * 2 + size + rad, windowPos.y + windowSize.y * 0.5f }
    };

    // Colors
    const ImU32 bgCol = ImGui::GetColorU32({ 0.2f, 0.2f, 0.2f, 0.7f });
    const ImU32 borderCol = ImGui::GetColorU32({ 0.5f, 0.5f, 0.5f, 0.8f });
    const ImU32 stickCol = armed
        ? ImGui::GetColorU32({ 0.2f, 0.8f, 0.2f, 1.0f })
        : ImGui::GetColorU32({ 0.8f, 0.2f, 0.2f, 1.0f });
    const ImU32 centerCol = ImGui::GetColorU32({ 0.7f, 0.7f, 0.7f, 0.5f });
    const ImU32 textCol = ImGui::GetColorU32({ 1.0f, 1.0f, 1.0f, 0.8f });

    // Draw both joysticks
    const char* labels[2] = { "Throttle/Yaw", "Pitch/Roll" };
    for (int i = 0; i < 2; ++i) {
        // Calculate background rectangle
        ImVec2 topLeft = ImVec2(centers[i].x - rad, centers[i].y - rad);
        ImVec2 bottomRight = ImVec2(centers[i].x + rad, centers[i].y + rad);

        // Draw filled rectangle
        dl->AddRectFilled(topLeft, bottomRight, bgCol, 3.0f);

        // Draw border
        dl->AddRect(topLeft, bottomRight, borderCol, 3.0f, 0, 2.0f);

        // Draw center marker
        dl->AddCircleFilled(centers[i], 2.0f, centerCol);

        // Calculate and draw stick position
        ImVec2 js = (i == 0 ? joystickState.leftStick : joystickState.rightStick);
        ImVec2 pos{
            centers[i].x + (js.x - 0.5f) * (size - stick * 2.0f),
            centers[i].y - (js.y - 0.5f) * (size - stick * 2.0f)
        };
        dl->AddCircleFilled(pos, stick, stickCol);

        // Draw label
        dl->AddText(
            { centers[i].x - 40.0f, centers[i].y + rad + 15.0f },
            textCol,
            labels[i]);
    }

    ImGui::End();
}

// Display battery warnings in center of screen
void renderBatteryWarnings() {
    // Only show warnings when battery is low
    float batteryPct = (battery.charge / battery.capacity) * 100.0f;
    if (batteryPct > 30.0f && !battery.isDepleted)
        return;

    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2{ 0.0f, 0.0f });
    ImGui::SetNextWindowSize(io.DisplaySize);

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoInputs |
        ImGuiWindowFlags_NoSavedSettings;

    ImGui::Begin("Battery Warnings", nullptr, flags);
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Default warning settings
    const char* msg = "BATTERY LOW";
    ImU32 color = ImGui::GetColorU32({ 1.0f, 0.5f, 0.0f, 1.0f }); // Orange
    float freq = 2.0f; // Pulsation frequency

    // Adjust warning based on battery level
    if (battery.isDepleted) {
        msg = "BATTERY EMPTY";
        color = ImGui::GetColorU32({ 1.0f, 0.0f, 0.0f, 1.0f }); // Red
        freq = 5.0f;
    }
    else if (batteryPct <= 10.0f) {
        msg = "BATTERY CRITICAL";
        color = ImGui::GetColorU32({ 1.0f, 0.0f, 0.0f, 1.0f }); // Red
        freq = 4.0f;
    }
    else if (batteryPct <= 20.0f) {
        msg = "BATTERY LOW!";
        color = ImGui::GetColorU32({ 1.0f, 0.2f, 0.0f, 1.0f }); // Orange-red
        freq = 3.0f;
    }

    // Create pulsating effect
    float t = static_cast<float>(glfwGetTime());
    float alpha = 0.5f + 0.5f * std::sin(t * freq);
    ImU32 col = (color & 0x00FFFFFF) | (static_cast<ImU32>(alpha * 255) << 24);

    // Text measurements
    float fontSize = 32.0f;
    ImVec2 txtSize = ImGui::CalcTextSize(msg);

    // Status line (percentage and voltage)
    char status[32];
    ImVec2 statusSize{ 0, 0 };
    if (!battery.isDepleted) {
        std::snprintf(status, sizeof(status), "%.1f%% (%.1fV)", batteryPct, battery.voltage);
        statusSize = ImGui::CalcTextSize(status);
    }

    // Calculate vertical centering
    float lineH = fontSize * 1.2f;
    float totalH = lineH + (battery.isDepleted ? 0.0f : statusSize.y + 10.0f);

    // Draw main warning text
    ImVec2 posMain{
        (io.DisplaySize.x - txtSize.x) * 0.5f,
        (io.DisplaySize.y - totalH) * 0.5f
    };

    // Draw text shadow
    dl->AddText(ImGui::GetFont(), fontSize, { posMain.x + 2, posMain.y + 2 },
        ImGui::GetColorU32({ 0.0f, 0.0f, 0.0f, alpha * 0.7f }), msg);

    // Draw main text
    dl->AddText(ImGui::GetFont(), fontSize, posMain, col, msg);

    // Draw status line if not depleted
    if (!battery.isDepleted) {
        ImVec2 posStat{
            (io.DisplaySize.x - statusSize.x) * 0.5f,
            posMain.y + lineH
        };

        // Draw status shadow
        dl->AddText({ posStat.x + 1, posStat.y + 1 },
            ImGui::GetColorU32({ 0.0f, 0.0f, 0.0f, alpha * 0.7f }), status);

        // Draw status text
        dl->AddText(posStat, col, status);
    }

    ImGui::End();
}

// Render the ImGui interface
void renderImGuiInterface() {
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Draw battery warnings on top of all windows
    renderBatteryWarnings();

    // Main control panel
    ImGui::Begin("Drone Control Panel");
    {
        ImGui::Text("Drone Status");
        ImGui::Separator();

        // Armed status indicator
        const ImVec4 statusColor = armed ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f)
            : ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
        ImGui::TextColored(statusColor, "Status: %s", armed ? "ARMED" : "DISARMED");

        // Arm/disarm button
        if (ImGui::Button(armed ? "DISARM" : "ARM")) {
            if (!armed && isDroneOnGround()) {
                armed = true;
            }
            else if (armed) {
                // Disarm and reset controls
                armed = false;
                controls.throttlePosition = 0.0f;
                input = {};
                rotationRates = glm::vec3(0.0f);
                for (auto& motor : motors) {
                    motor.targetThrottle = 0.0f;
                    motor.currentThrottle = 0.0f;
                    motor.isSpinning = false;
                    motor.thrust = 0.0f;
                }
            }
        }

        ImGui::Separator();

        // Battery info with progress bar
        const float batteryPct = (battery.charge / battery.capacity) * 100.0f;
        ImGui::Text("Battery: %.1f%% (%.1fV)", batteryPct, battery.voltage);
        ImGui::ProgressBar(batteryPct / 100.0f, ImVec2(-1.0f, 0.0f), "");

        // Battery type selector
        const char* types[] = { "1S", "2S", "3S", "4S", "5S", "6S" };
        int currentType = static_cast<int>(droneConfig.batteryType);
        if (ImGui::Combo("Battery Type", &currentType, types, IM_ARRAYSIZE(types))) {
            droneConfig.batteryType = static_cast<BatteryType>(currentType);
            droneConfig.updateMinBatteryVoltage();
            battery.configureBattery(droneConfig.batteryType);
            initializeDrone();
        }

        ImGui::Separator();

        // Motor information
        if (ImGui::CollapsingHeader("Motors", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (int i = 0; i < NUM_MOTORS; ++i) {
                ImGui::PushID(i);
                ImGui::Text("Motor %d: %.0f RPM", i + 1, motors[i].currentRPM);
                ImGui::ProgressBar(motors[i].currentThrottle, ImVec2(-1.0f, 0.0f), "");
                ImGui::PopID();
            }
        }

        ImGui::Separator();

        // Telemetry data
        if (ImGui::CollapsingHeader("Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) {
            const glm::vec3 euler = quatToEuler(droneQuat);
            ImGui::Text("Position: X:%.2f Y:%.2f Z:%.2f",
                dronePosition.x, dronePosition.y, dronePosition.z);
            ImGui::Text("Angles: Roll:%.1f° Pitch:%.1f° Yaw:%.1f°",
                euler.z, euler.x, euler.y);
            const float speedKmh = glm::length(velocity) * MS_TO_KMH;
            ImGui::Text("Speed: %.1f km/h", speedKmh);
        }

        ImGui::Separator();

        // Reset button
        if (ImGui::Button("Reset Drone")) {
            initializeDrone();
        }
    }
    ImGui::End();

    // Joystick controls in separate window
    renderJoystickControls();

    // Render all ImGui draw data
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// Initialize ImGui
void initImGui(GLFWwindow* window) {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    const char* glsl_version = "#version 330";
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load default font
    io.Fonts->AddFontDefault();
}

// Shutdown ImGui
void shutdownImGui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

//-------------------------------------------------------
// Main Function
//-------------------------------------------------------

int main() {
    // Initialize GLFW
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create window
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "FPV Drone Simulator", nullptr, nullptr);
    if (!window) {
        std::cerr << "ERROR: GLFW window creation failed\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize GLAD
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "ERROR: Failed to initialize GLAD\n";
        return -1;
    }

    // Print OpenGL info
    getOpenGLInfo();
    std::cout << "OpenGL: " << glVersionString << "\n"
        << "GLSL  : " << glslVersionString << "\n";

    // Initialize simulation
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    initializeDrone();
    setupShaders();
    setupFloorBuffers();
    setupSkyboxBuffers();

    // Initialize ImGui
    initImGui(window);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Process input and update simulation
        processInput(window);

        // Calculate FPS
        const float dt = (deltaTime > 1e-4f ? deltaTime : 1e-4f);
        const float fps = 1.0f / dt;

        // Clear screen
        glClearColor(0.95f, 0.65f, 0.35f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Create projection matrix
        const float aspect = static_cast<float>(currentWidth) / currentHeight;
        glm::mat4 projection = glm::perspective(glm::radians(75.0f), aspect, 0.1f, 2000.0f);

        // Setup FPV camera
        const float baseHeight = droneFrame.frameHeightMeters * 0.6f;
        const float forwardOffset = droneFrame.frameSizeMeters * 0.15f;
        glm::vec3 localOffset(0.0f, baseHeight, -forwardOffset);
        auto cameraOffset = glm::rotate(droneQuat, localOffset);
        auto camPos = dronePosition + cameraOffset;

        // Ensure camera doesn't clip through ground
        constexpr float minHeight = 0.15f;
        float downwardComp = 0.0f;
        if (cameraAngle < 0.0f) {
            float lookDist = std::abs(std::tan(glm::radians(cameraAngle))) * 0.5f;
            downwardComp = lookDist + std::abs(cameraAngle) * 0.01f;
        }
        camPos.y = std::max(camPos.y, minHeight + downwardComp);

        // Apply FPV camera angle
        glm::quat rotFPV = glm::angleAxis(glm::radians(cameraAngle), glm::vec3(1.0f, 0.0f, 0.0f));
        glm::quat combinedRot = droneQuat * rotFPV;

        // Fix extreme downward angles
        if (cameraAngle < -30.0f) {
            glm::vec3 viewDir = glm::rotate(combinedRot, glm::vec3(0.0f, 0.0f, -1.0f));
            if (viewDir.y < -0.9f) {
                float corrAngle = (viewDir.y + 0.9f) * 15.0f;
                glm::quat fixRot = glm::angleAxis(glm::radians(corrAngle), glm::vec3(1.0f, 0.0f, 0.0f));
                combinedRot = fixRot * combinedRot;
            }
        }

        // Create view matrix
        glm::mat4 camTransform = glm::translate(glm::mat4(1.0f), camPos) *
            glm::mat4_cast(combinedRot);
        glm::mat4 view = glm::inverse(camTransform);

        // Render scene
        drawSkybox(projection, glm::mat4(glm::mat3(view))); // Skybox without translation
        drawFloor(projection, view);

        // Render UI
        renderImGuiInterface();
        updateWindowTitle(window, fps);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    shutdownImGui();
    glDeleteProgram(shaderProgram);
    glDeleteVertexArrays(1, &floorVAO);
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteVertexArrays(1, &axesVAO);
    glDeleteVertexArrays(1, &skyboxVAO);
    glDeleteBuffers(1, &floorVBO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteBuffers(1, &axesVBO);
    glDeleteBuffers(1, &skyboxVBO);

    glfwTerminate();
    return 0;
}