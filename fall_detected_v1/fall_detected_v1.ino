// First define your Blynk configuration
#define BLYNK_TEMPLATE_ID "TMPL6axjoRGTv"
#define BLYNK_TEMPLATE_NAME "Fall Detected"
#define BLYNK_AUTH_TOKEN "k51g17_Pi_pBQxeMUEFqyBeVn1RDB-At"
#define BLYNK_PRINT Serial             // In log Blynk qua Serial

// Then include the libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>  // Include Blynk library after defining template ID and name

// ==== BLYNK CONFIGURATION ====
char auth[] = BLYNK_AUTH_TOKEN;        // Blynk Auth Token
char ssid[] = "Asus";                  // Tên WiFi của bạn
char pass[] = "trangckun";             // Mật khẩu WiFi

// ==== MPU6050 CONFIGURATION ====
Adafruit_MPU6050 mpu;

// Ngưỡng phát hiện té ngã
const float fallThreshold = 2.0;  // Gia tốc lớn hơn 2 m/s^2
const float gyroThreshold = 5.0;  // Tốc độ quay lớn hơn 5 rad/s

bool fallDetected = false;        // Biến cờ để theo dõi té ngã

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Khởi động thiết bị...");

  Wire.begin(2, 15);
  Serial.println("I2C được khởi tạo trên SDA=2, SCL=15");

  // // Kết nối WiFi trước
  // Serial.print("Đang kết nối WiFi...");
  // WiFi.begin(ssid, pass);
  
  // int wifiAttempts = 0;
  // while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
  //   delay(500);
  //   Serial.print(".");
  //   wifiAttempts++;
  // }
  
  // if (WiFi.status() == WL_CONNECTED) {
  //   Serial.println();
  //   Serial.println("WiFi đã kết nối!");
  //   Serial.print("IP: ");
  //   Serial.println(WiFi.localIP());
    
  //   // Sau khi WiFi kết nối, kết nối Blynk
  //   Serial.println("Đang kết nối Blynk...");
  //   Blynk.config(auth);
    
  //   if (Blynk.connect()) {
  //     Serial.println("Blynk đã kết nối thành công!");
  //   } else {
  //     Serial.println("Không thể kết nối đến Blynk. Kiểm tra Auth Token.");
  //   }
  // } else {
  //   Serial.println();
  //   Serial.println("Không thể kết nối WiFi. Kiểm tra cài đặt WiFi.");
  // }

  // Khởi tạo MPU6050
  if (!mpu.begin()) {
    Serial.println("Không tìm thấy MPU6050! Kiểm tra kết nối.");
    while (1) delay(100);
  }
  Serial.println("MPU6050 đã khởi động thành công!");
}

void loop() {
  // Blynk.run();  // Luôn gọi hàm này để duy trì kết nối với Blynk

  // Đọc dữ liệu từ cảm biến
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // In chi tiết gia tốc từng trục
  Serial.print("Accel (m/s^2) -> X: "); Serial.print(accel.acceleration.x, 3);
  Serial.print(" Y: "); Serial.print(accel.acceleration.y, 3);
  Serial.print(" Z: "); Serial.println(accel.acceleration.z, 3);

  // In chi tiết gyro từng trục
  Serial.print("Gyro (rad/s) -> X: "); Serial.print(gyro.gyro.x, 3);
  Serial.print(" Y: "); Serial.print(gyro.gyro.y, 3);
  Serial.print(" Z: "); Serial.println(gyro.gyro.z, 3);

  // Tính tổng vector gia tốc và tốc độ quay
  float totalAcceleration = sqrt(
    pow(accel.acceleration.x, 2) +
    pow(accel.acceleration.y, 2) +
    pow(accel.acceleration.z, 2)
  );

  float totalGyro = sqrt(
    pow(gyro.gyro.x, 2) +
    pow(gyro.gyro.y, 2) +
    pow(gyro.gyro.z, 2)
  );

  Serial.print("Tổng gia tốc: ");
  Serial.print(totalAcceleration, 3);
  Serial.print(" m/s^2, Tổng gyro: ");
  Serial.print(totalGyro, 3);
  Serial.println(" rad/s");

  // Phát hiện té ngã
  if (totalAcceleration > fallThreshold && totalGyro > gyroThreshold && !fallDetected) {
    Serial.println("⚠️ Té ngã được phát hiện!");
    // Blynk.logEvent("fall");  // Gửi log event tới Blynk Cloud
    fallDetected = true;
  } 
  else if (totalAcceleration < fallThreshold && totalGyro < gyroThreshold) {
    // Reset cờ nếu không có té ngã nữa
    if (fallDetected) {
      Serial.println("Tình trạng bình thường trở lại.");
    }
    fallDetected = false;
  } else {
    Serial.println("Tình trạng bình thường.");
  }

  delay(1000);  // Delay 1s giữa mỗi lần đọc
}