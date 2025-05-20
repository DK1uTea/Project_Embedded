
#include <TensorFlowLite_ESP32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model_data.h"

#define BUZZER_PIN 14
#define BUTTON_PIN 12
#define WINDOW_SIZE 50

Adafruit_MPU6050 mpu;
float accel_x_buffer[WINDOW_SIZE];
float accel_y_buffer[WINDOW_SIZE];
float accel_z_buffer[WINDOW_SIZE];
int buffer_index = 0;
bool buffer_filled = false;
bool fall_detected = false;
bool alarm_active = false;
unsigned long alarm_start_time = 0;
const unsigned long alarm_duration = 10000;

tflite::MicroErrorReporter micro_error_reporter;
tflite::ErrorReporter* error_reporter = &micro_error_reporter;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
constexpr int kTensorArenaSize = 8 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
float features[18];

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);
  
  Wire.begin(2, 15);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  for (int i = 0; i < WINDOW_SIZE; i++) {
    accel_x_buffer[i] = 0;
    accel_y_buffer[i] = 0;
    accel_z_buffer[i] = 0;
  }
  
  model = tflite::GetModel(model_data);
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
    model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;
  
  interpreter->AllocateTensors();
  input = interpreter->input(0);
  output = interpreter->output(0);
  
  Serial.println("Fall detection system initialized");
}

void loop() {
  int button_state = digitalRead(BUTTON_PIN);
  
  if (button_state == LOW && alarm_active) {
    deactivateAlarm();
  }
  
  if (alarm_active && (millis() - alarm_start_time > alarm_duration)) {
    deactivateAlarm();
  }
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accel_x_buffer[buffer_index] = a.acceleration.x;
  accel_y_buffer[buffer_index] = a.acceleration.y;
  accel_z_buffer[buffer_index] = a.acceleration.z;
  
  buffer_index = (buffer_index + 1) % WINDOW_SIZE;
  
  if (buffer_index == 0 && !buffer_filled) {
    buffer_filled = true;
    Serial.println("Buffer filled, fall detection active");
  }
  
  if (buffer_filled) {
    extract_features(accel_x_buffer, accel_y_buffer, accel_z_buffer, WINDOW_SIZE, features);
    bool is_fall = detect_fall_tflite(features);
    
    if (is_fall && !fall_detected) {
      Serial.println("FALL DETECTED!");
      fall_detected = true;
      activateAlarm();
    } 
    else if (!is_fall && fall_detected) {
      fall_detected = false;
      Serial.println("No fall detected");
    }
  }
  
  static int counter = 0;
  if (counter++ % 10 == 0) {
    Serial.print("Accel (m/s^2): X: ");
    Serial.print(a.acceleration.x, 1);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y, 1);
    Serial.print(", Z: ");
    Serial.println(a.acceleration.z, 1);
  }
  
  delay(20);
}

void extract_features(float accel_x[], float accel_y[], float accel_z[], int window_size, float features[]) {
  float mean_x = 0, mean_y = 0, mean_z = 0;
  float min_x = accel_x[0], min_y = accel_y[0], min_z = accel_z[0];
  float max_x = accel_x[0], max_y = accel_y[0], max_z = accel_z[0];
  
  for (int i = 0; i < window_size; i++) {
    mean_x += accel_x[i];
    mean_y += accel_y[i];
    mean_z += accel_z[i];
    
    if (accel_x[i] < min_x) min_x = accel_x[i];
    if (accel_y[i] < min_y) min_y = accel_y[i];
    if (accel_z[i] < min_z) min_z = accel_z[i];
    
    if (accel_x[i] > max_x) max_x = accel_x[i];
    if (accel_y[i] > max_y) max_y = accel_y[i];
    if (accel_z[i] > max_z) max_z = accel_z[i];
  }
  
  mean_x /= window_size;
  mean_y /= window_size;
  mean_z /= window_size;
  
  float var_x = 0, var_y = 0, var_z = 0;
  for (int i = 0; i < window_size; i++) {
    var_x += (accel_x[i] - mean_x) * (accel_x[i] - mean_x);
    var_y += (accel_y[i] - mean_y) * (accel_y[i] - mean_y);
    var_z += (accel_z[i] - mean_z) * (accel_z[i] - mean_z);
  }
  
  var_x /= window_size;
  var_y /= window_size;
  var_z /= window_size;
  
  float std_x = sqrt(var_x);
  float std_y = sqrt(var_y);
  float std_z = sqrt(var_z);
  
  float mag_values[window_size];
  float mean_mag = 0, min_mag = 0, max_mag = 0;
  
  for (int i = 0; i < window_size; i++) {
    mag_values[i] = sqrt(accel_x[i]*accel_x[i] + accel_y[i]*accel_y[i] + accel_z[i]*accel_z[i]);
    mean_mag += mag_values[i];
    
    if (i == 0 || mag_values[i] < min_mag) min_mag = mag_values[i];
    if (i == 0 || mag_values[i] > max_mag) max_mag = mag_values[i];
  }
  
  mean_mag /= window_size;
  
  int idx = 0;
  features[idx++] = mean_x;
  features[idx++] = std_x;
  features[idx++] = min_x;
  features[idx++] = max_x;
  features[idx++] = max_x - min_x;
  
  features[idx++] = mean_y;
  features[idx++] = std_y;
  features[idx++] = min_y;
  features[idx++] = max_y;
  features[idx++] = max_y - min_y;
  
  features[idx++] = mean_z;
  features[idx++] = std_z;
  features[idx++] = min_z;
  features[idx++] = max_z;
  features[idx++] = max_z - min_z;
  
  features[idx++] = mean_mag;
  features[idx++] = max_mag;
  features[idx++] = max_mag - min_mag;
}

bool detect_fall_tflite(float feature_array[]) {
  for (int i = 0; i < 18; i++) {
    input->data.f[i] = feature_array[i];
  }
  
  interpreter->Invoke();
  float fall_probability = output->data.f[0];
  
  return fall_probability > 0.5;
}

void activateAlarm() {
  alarm_active = true;
  alarm_start_time = millis();
  digitalWrite(BUZZER_PIN, HIGH);
  Serial.println("ALARM ACTIVATED!");
}

void deactivateAlarm() {
  alarm_active = false;
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Alarm deactivated");
}
