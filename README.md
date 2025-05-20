# AI-Powered Fall Detection System (ESP32 & MPU6050)

## Overview

This project implements a real-time fall detection system using an ESP32 microcontroller, an MPU6050 accelerometer/gyroscope, and a pre-trained AI model. The system continuously monitors movement data from the MPU6050, classifies activities using an AI model, and triggers a local alarm (buzzer) and sends a notification via the Blynk mobile application upon detecting a fall.

The system is designed to identify different types of activities, including various fall patterns, and provide timely alerts.

**Note: The final version of this project is designated as fall_detection_v5_AI.**

## Features

*   **Real-time Fall Detection:** Utilizes MPU6050 sensor data (accelerometer and gyroscope) for immediate motion analysis.
*   **AI-Based Activity Classification:** Employs a Random Forest model (hardcoded decision trees) to classify activities. Detected activities include:
    *   "Ngồi xuống" (Sitting down)
    *   "Ngã tự do" (Free Fall)
    *   "Ngã khi chạy" (Running Fall)
    *   "Chạy và ngồi" (Running and Sitting)
    *   "Ngã khi đi bộ" (Walking Fall)
    *   "Đi bộ và ngồi" (Walking and Sitting)
*   **Blynk IoT Platform Integration:**
    *   Remote notifications of fall events.
    *   System status monitoring (online/offline).
    *   Real-time display of sensor readings (total acceleration, total gyroscope).
    *   Detailed fall event logging (fall type, timestamp, sensor values).
    *   Display of currently detected activity type.
*   **Local Alerts:**
    *   Audible alarm using a buzzer.
    *   Manual alarm deactivation via a push button.
*   **Status Indication:** LED for visual feedback on system operation.
*   **Connectivity:**
    *   WiFi connection for Blynk communication.
    *   Automatic reconnection attempts for WiFi and Blynk.
*   **Configurable:** WiFi credentials and Blynk authentication tokens can be easily set in the main `.ino` file.
*   **Debug Mode:** Provides detailed serial output for testing and troubleshooting.

## Hardware Requirements

*   ESP32 Development Board
*   MPU6050 Accelerometer and Gyroscope Module
*   Active Buzzer
*   Push Button
*   LED (e.g., 5mm LED)
*   Current-limiting resistor for the LED (e.g., 220-330 Ohm)
*   Connecting Wires
*   Breadboard (optional, for prototyping)
*   Power Supply (e.g., USB Micro-B or USB-C, depending on your ESP32 board)

## Software & Libraries

*   **Arduino IDE:** Version 1.8.13 or higher.
*   **ESP32 Board Support Package:** For Arduino IDE (e.g., from Espressif).
*   **Libraries (install via Arduino Library Manager):**
    *   `Adafruit MPU6050` by Adafruit
    *   `Adafruit Unified Sensor` by Adafruit (dependency for MPU6050 library)
    *   `Blynk` by Volodymyr Shymanskyy
    *   `Wire` (usually built-in with ESP32 core)
    *   `WiFi` (built-in with ESP32 core)
*   **Custom AI Model Files:**
    *   `fall_detection_model.h`
    *   `fall_detection_model.cpp` (These should be in the same directory as `fall_detection_v5_AI.ino`)

## Setup Instructions

### 1. Hardware Connections

*   **MPU6050 Module:**
    *   `VCC` to `3.3V` on ESP32
    *   `GND` to `GND` on ESP32
    *   `SCL` to `GPIO15` on ESP32 (as per `Wire.begin(2, 15);`)
    *   `SDA` to `GPIO2` on ESP32 (as per `Wire.begin(2, 15);`)
*   **Buzzer:**
    *   Positive (`+`) leg to `GPIO14` on ESP32 (`BUZZER_PIN`)
    *   Negative (`-`) leg to `GND` on ESP32
*   **Push Button:**
    *   One leg to `GPIO12` on ESP32 (`BUTTON_PIN`)
    *   Other leg to `GND` on ESP32 (The code uses `INPUT_PULLUP`, so an external pull-up resistor is not strictly needed if the button connects to GND when pressed).
*   **LED:**
    *   Anode (longer leg) to `GPIO2` on ESP32 (`LED_PIN`) via a current-limiting resistor (e.g., 220 Ohms). (Note: `LED_PIN` is defined as `2`, which is also used for MPU6050 SDA. This is a conflict. **You should change `LED_PIN` to an unused GPIO pin, for example, `GPIO4` or `GPIO13` etc.**)
    *   Cathode (shorter leg) to `GND` on ESP32.

**Important Pin Conflict:** The default `LED_PIN` (GPIO2) conflicts with the MPU6050 SDA pin (GPIO2). Please change `LED_PIN` in `fall_detection_v5_AI.ino` to an unused GPIO pin.

### 2. Blynk App Setup

1.  Download the Blynk app (iOS or Android).
2.  Create a new Blynk account or log in.
3.  Create a new Project or use the provided template details:
    *   Tap "New Project".
    *   Give your project a name.
    *   Choose **ESP32 Dev Board** as the device.
    *   Connection Type: **WiFi**.
    *   Theme: Dark or Light.
    *   You will receive an **Auth Token** via email. You'll need this for the Arduino code.
    *   Alternatively, if you have the Template ID and Name, you might be able to provision from a template. The code specifies:
        *   `BLYNK_TEMPLATE_ID "TMPL6GPaA0o6d"`
        *   `BLYNK_TEMPLATE_NAME "Fall Detection"`
4.  Add the following widgets to your project dashboard, connected to the specified Virtual Pins:
    *   **Notification:** To receive fall alerts.
    *   **Labeled Value / Value Display (VPIN_SYSTEM_STATUS - V1):** For System Status (e.g., "Thiết bị trực tuyến").
    *   **Labeled Value / Value Display (VPIN_FALL_DETECTION - V0):** For Fall Detection Status (e.g., "⚠️ TÉ NGÃ ĐƯỢC PHÁT HIỆN!").
    *   **Gauge / Labeled Value (VPIN_ACCEL_TOTAL - V3):** For Total Acceleration.
    *   **Gauge / Labeled Value (VPIN_GYRO_TOTAL - V4):** For Total Gyroscope.
    *   **Terminal / Labeled Value (VPIN_FALL_DETAIL - V5):** For Detailed Fall Information.
    *   **Labeled Value / Value Display (VPIN_ACTIVITY_TYPE - V6):** For Activity Type.

### 3. Arduino IDE Setup

1.  Install the Arduino IDE.
2.  Install the ESP32 board support package. (You can find instructions by searching "ESP32 Arduino Core installation").
3.  Install the required libraries (see [Software & Libraries](#software--libraries) section) via `Sketch > Include Library > Manage Libraries...`.
4.  Open the `fall_detection_v5_AI.ino` file in the Arduino IDE.
5.  Ensure `fall_detection_model.h` and `fall_detection_model.cpp` are in the same folder as the `.ino` file.

### 4. Code Configuration

Open `fall_detection_v5_AI.ino` and update the following:

*   **Blynk Credentials:**
    ```c++
    #define BLYNK_TEMPLATE_ID "TMPL6GPaA0o6d" // Or your new Template ID
    #define BLYNK_TEMPLATE_NAME "Fall Detection" // Or your new Template Name
    #define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN" // Paste your Auth Token here
    ```
*   **WiFi Credentials:**
    ```c++
    char ssid[] = "YOUR_WIFI_SSID";         // Your WiFi network name
    char pass[] = "YOUR_WIFI_PASSWORD";     // Your WiFi password
    ```
*   **(Crucial) LED Pin:** Change `LED_PIN` if you haven't already to avoid conflict with MPU6050 SDA. For example:
    ```c++
    #define LED_PIN 4 // Example: Using GPIO4 for the LED
    ```
*   **(Optional) Other Pins:** If your hardware connections differ, update `BUZZER_PIN`, `BUTTON_PIN`, and the I2C pins in `Wire.begin()`.

### 5. Uploading the Code

1.  Select the correct ESP32 board from `Tools > Board`.
2.  Select the correct COM port from `Tools > Port`.
3.  Click the "Upload" button in the Arduino IDE.

## How it Works

1.  **Initialization:**
    *   Sets up GPIO pins for the buzzer, button, and LED.
    *   Initializes I2C communication and the MPU6050 sensor.
    *   Sets the MPU6050 accelerometer range (±8g).
    *   Initializes data buffers (`accel_x_buffer`, `accel_y_buffer`, `accel_z_buffer`) with a size of `WINDOW_SIZE` (50 samples).
    *   Connects to the specified WiFi network and then to the Blynk server.
2.  **Data Acquisition & Buffering:**
    *   In the main loop, the system continuously reads accelerometer (X, Y, Z) and gyroscope data from the MPU6050.
    *   The accelerometer data (X, Y, Z) is stored in circular buffers. Each buffer holds the most recent `WINDOW_SIZE` samples.
3.  **Feature Extraction:**
    *   Once a buffer is filled (`buffer_filled = true`), statistical features are extracted from the data in the window. This is done by the `extract_features` function in `fall_detection_model.cpp`.
    *   The 18 extracted features include:
        *   Mean, Standard Deviation, Min, Max, and Range (Max-Min) for each accelerometer axis (X, Y, Z).
        *   Mean, Max, and Range for the magnitude of the accelerometer vector.
4.  **AI Model Prediction:**
    *   The array of 18 extracted features is passed to the `detect_activity` function in `fall_detection_model.cpp`.
    *   This function uses a set of five hardcoded decision trees (`tree_0` to `tree_4`) which form a Random Forest classifier.
    *   Each tree votes for an activity class, and the class with the most votes is chosen as the predicted activity.
    *   **Critical Note:** The current AI model (`tree_x` functions) references feature indices far beyond the 18 features extracted (e.g., `features[84]`). This is a significant issue and means the model will likely not perform as intended. See [Known Issues](#known-issues--important-notes).
5.  **Fall Detection Logic:**
    *   The system checks if the predicted `activity` ID is present in the `fall_activity_ids` array (currently `{1, 2}`, corresponding to "Ngã tự do" (freeFall) and "Ngã khi chạy" (runFall)).
    *   If a fall activity is detected and no previous fall is currently active (`!fallDetected`):
        *   The `fallDetected` flag is set to `true`.
        *   The buzzer is activated (`activateAlarm()`).
        *   A notification is sent to the Blynk app, including the fall type, a unique event ID, timestamp, and current sensor readings.
        *   Detailed fall information is also sent to a separate virtual pin on Blynk.
6.  **Alerting & Reset:**
    *   **Buzzer:** The buzzer sounds for `alarmDuration` (10 seconds) or until the user presses the `BUTTON_PIN`.
    *   **Button:** Pressing the button deactivates the buzzer (`deactivateAlarm()`).
    *   **Blynk:** Updates virtual pins with fall status, sensor data, and activity type.
    *   **Auto-Reset:** The `fallDetected` status automatically resets to normal after `resetTimeout` (60 seconds), and a "Bình thường" (Normal) status is sent to Blynk.
7.  **Connectivity & Status:**
    *   The system periodically checks WiFi and Blynk connectivity. If disconnected, it attempts to reconnect at `reconnectInterval` (30 seconds).
    *   The onboard LED blinks to indicate the system is operational.
    *   Total acceleration and gyroscope values are sent to Blynk every second.

## Files in the Project

*   `fall_detection_v5_AI/fall_detection_v5_AI.ino`: The main Arduino sketch. Contains `setup()`, `loop()`, sensor data handling, Blynk communication, fall detection logic, and peripheral control.
*   `fall_detection_v5_AI/fall_detection_model.cpp`: Implements the feature extraction function (`extract_features`) and the AI model (Random Forest classifier as `tree_x` functions and `detect_activity`).
*   `fall_detection_v5_AI/fall_detection_model.h`: Header file declaring the functions in `fall_detection_model.cpp`.

## Known Issues & Important Notes

*   🚨 **Critical AI Model Feature Mismatch:**
    The `extract_features` function calculates and populates an array with 18 distinct features. However, the decision tree functions (`tree_0` through `tree_4` in `fall_detection_model.cpp`) access feature indices far beyond this range (e.g., `features[84]`, `features[92]`, `features[58]`). This means the AI model is attempting to use data that hasn't been calculated or is out of the bounds of the `features` array, which will lead to **unreliable and incorrect activity predictions**.
    *   **To fix this:** The decision tree logic in `fall_detection_model.cpp` needs to be revised to *only* use indices from 0 to 17, corresponding to the 18 features actually extracted. Alternatively, if the model genuinely requires more features, the `extract_features` function must be updated to calculate them.
*   **LED Pin Conflict:** As mentioned in the hardware setup, the default `LED_PIN` (GPIO2) conflicts with the MPU6050 SDA pin. **This must be changed** to an unused GPIO pin for both the MPU6050 and LED to function correctly.
*   **Fall Type Definition Inconsistency:**
    *   The `fall_activity_ids` array in `fall_detection_v5_AI.ino` is `const int fall_activity_ids[] = {1, 2};`. This corresponds to "Ngã tự do" (freeFall - ID 1) and "Ngã khi chạy" (runFall - ID 2).
    *   However, comments in the code (e.g., `// Loại té ngã phát hiện: freeFall, runFall, walkFall` in `setup()` and the comment next to `fall_activity_ids` definition) suggest that "Ngã khi đi bộ" (walkFall - ID 4) might also be an intended fall type.
    *   To include "Ngã khi đi bộ" (walkFall) as a detected fall, update the array to: `const int fall_activity_ids[] = {1, 2, 4};`.
*   **"Safety Checks" in Model Code:** The `fall_detection_model.cpp` file includes comments like `// Safety check: Index out of bounds` within the tree functions (e.g., `tree_1`). This further suggests that the porting or definition of the decision tree model might have inherent issues or was intended for a different feature set.
*   **Language:** Most serial print messages and some Blynk messages are in Vietnamese.

## Customization

*   **WiFi & Blynk Credentials:** Modify in `fall_detection_v5_AI.ino`.
*   **Sensor Sensitivity:** The MPU6050 range is set via `mpu.setAccelerometerRange(MPU6050_RANGE_8_G);`. This can be changed to other supported ranges if needed.
*   **AI Model Window Size:** `WINDOW_SIZE` (default 50) can be adjusted, but this would likely require retraining the AI model.
*   **AI Model Logic:** To use a different or corrected AI model:
    1.  Train your model (e.g., Random Forest, Decision Tree) using your dataset.
    2.  Convert the trained model into C/C++ code (e.g., a series of if-else statements for decision trees).
    3.  Replace or modify the `tree_X` functions and potentially the `detect_activity` logic in `fall_detection_model.cpp`. Ensure the feature extraction matches the model's input requirements.
*   **Timers:** Adjust `reconnectInterval`, `resetTimeout`, `alarmDuration` in `fall_detection_v5_AI.ino` as needed.

## Troubleshooting

*   **No Serial Output:** Check baud rate (115200), COM port selection, and board selection in Arduino IDE. Ensure ESP32 is powered and connected.
*   **MPU6050 Not Found:** Verify I2C wiring (SDA, SCL, VCC, GND). Ensure the correct I2C pins are used in `Wire.begin()`. Check for pin conflicts.
*   **WiFi Connection Issues:** Double-check SSID and password. Ensure your WiFi network is 2.4GHz, as ESP32s often have issues with 5GHz networks. Check WiFi signal strength.
*   **Blynk Connection Issues:** Verify Auth Token. Ensure Blynk server is reachable (default is `blynk.cloud`).
*   **Incorrect Fall Detection / Activity Classification:** This is highly likely due to the **Critical AI Model Feature Mismatch** mentioned in [Known Issues](#known-issues--important-notes). This needs to be addressed first.
*   **LED Not Working:** Check wiring, resistor, and ensure `LED_PIN` is correctly defined and not conflicting.

---

This README should give a good starting point for anyone looking to understand or use your project. Remember to address the critical issues, especially the AI model feature mismatch and the LED pin conflict, for the project to work as intended!