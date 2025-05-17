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

// ==== PINS CONFIGURATION ====
#define BUZZER_PIN 14        // Chân còi kết nối với cổng 14
#define BUTTON_PIN 12        // Chân nút nhấn kết nối với cổng 12

// ==== BLYNK CONFIGURATION ====
char auth[] = BLYNK_AUTH_TOKEN;        // Blynk Auth Token
char ssid[] = "Asus";                  // Tên WiFi của bạn
char pass[] = "trangckun";             // Mật khẩu WiFi

// ==== MPU6050 CONFIGURATION ====
Adafruit_MPU6050 mpu;

// Ngưỡng phát hiện té ngã
const float GRAVITY = 9.81;           // Gia tốc trọng trường
const float fallThreshold = 15.0;     // Ngưỡng gia tốc (so với trọng lực)
const float gyroThreshold = 3.0;      // Ngưỡng tốc độ quay

// Biến để lưu giá trị tối đa cho mục đích kiểm tra
float maxAcceleration = 0;
float maxGyro = 0;

// Biến trạng thái
bool fallDetected = false;        // Biến cờ để theo dõi té ngã
bool alarmActive = false;         // Biến theo dõi trạng thái còi báo động
bool testMode = true;             // Chế độ kiểm tra - hiển thị thông tin debug

// Biến xử lý còi
unsigned long alarmStartTime = 0;
const unsigned long alarmDuration = 10000;  // Thời gian còi kêu tối đa (10 giây)

// Biến xử lý nút nhấn
int lastButtonState = HIGH;       // Trạng thái trước đó của nút
unsigned long lastButtonPressTime = 0; // Thời điểm nhấn nút cuối cùng

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Khởi động thiết bị...");

  // Cài đặt chân GPIO
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Sử dụng điện trở kéo lên
  
  // Đảm bảo còi tắt khi khởi động
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(2, 15);
  Serial.println("I2C được khởi tạo trên SDA=2, SCL=15");

  // Khởi tạo MPU6050
  if (!mpu.begin()) {
    Serial.println("Không tìm thấy MPU6050! Kiểm tra kết nối.");
    while (1) delay(100);
  }
  Serial.println("MPU6050 đã khởi động thành công!");
  
  // Hiển thị thông tin về ngưỡng kiểm tra
  Serial.println("\n--- HỆ THỐNG PHÁT HIỆN TÉ NGÃ ---");
  Serial.println("Chân còi: " + String(BUZZER_PIN) + ", Chân nút nhấn: " + String(BUTTON_PIN));
  Serial.print("Ngưỡng gia tốc: "); Serial.print(fallThreshold); Serial.println(" m/s^2");
  Serial.print("Ngưỡng gyro: "); Serial.print(gyroThreshold); Serial.println(" rad/s");
  Serial.println("Di chuyển thiết bị để mô phỏng té ngã.");
  Serial.println("Nhấn nút để tắt còi báo động khi kích hoạt.");
  Serial.println("------------------------------\n");
}

void loop() {
  // Đọc trạng thái nút nhấn NGAY ĐẦU VÒNG LẶP để đảm bảo phản hồi nhanh
  int buttonState = digitalRead(BUTTON_PIN);
  
  // Hiển thị trạng thái nút nhấn để debug
  if (buttonState != lastButtonState) {
    Serial.println("Trạng thái nút: " + String(buttonState == LOW ? "NHẤN" : "NHẢ"));
    lastButtonState = buttonState;
    
    // Khi nút được nhấn (LOW với INPUT_PULLUP)
    if (buttonState == LOW) {
      // Ghi nhớ thời gian nhấn nút
      lastButtonPressTime = millis();
      
      // Nếu còi đang kêu, tắt còi ngay lập tức
      if (alarmActive) {
        Serial.println("NÚT NHẤN ĐƯỢC PHÁT HIỆN - TẮT CÒI!");
        deactivateAlarm();
      }
    }
  }
  
  // Kiểm tra và quản lý còi báo động
  if (alarmActive && (millis() - alarmStartTime > alarmDuration)) {
    deactivateAlarm();
    Serial.println("Còi báo động đã tự động tắt sau " + String(alarmDuration/1000) + " giây.");
  }

  // Đọc dữ liệu từ cảm biến
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

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

  // Tính sự thay đổi gia tốc (loại bỏ trọng lực)
  float accelerationChange = abs(totalAcceleration - GRAVITY);

  // Cập nhật giá trị tối đa
  if (totalAcceleration > maxAcceleration) maxAcceleration = totalAcceleration;
  if (totalGyro > maxGyro) maxGyro = totalGyro;

  // In ra thông tin debug trong chế độ kiểm tra
  if (testMode) {
    // In chi tiết gia tốc từng trục
    Serial.print("Accel (m/s^2) -> X: "); Serial.print(accel.acceleration.x, 3);
    Serial.print(" Y: "); Serial.print(accel.acceleration.y, 3);
    Serial.print(" Z: "); Serial.println(accel.acceleration.z, 3);

    // In chi tiết gyro từng trục
    Serial.print("Gyro (rad/s) -> X: "); Serial.print(gyro.gyro.x, 3);
    Serial.print(" Y: "); Serial.print(gyro.gyro.y, 3);
    Serial.print(" Z: "); Serial.println(gyro.gyro.z, 3);

    // In ra các tổng giá trị và % so với ngưỡng
    Serial.print("Tổng gia tốc: ");
    Serial.print(totalAcceleration, 3);
    Serial.print(" m/s^2 (");
    Serial.print(int(totalAcceleration/fallThreshold*100));
    Serial.print("% ngưỡng), Tổng gyro: ");
    Serial.print(totalGyro, 3);
    Serial.print(" rad/s (");
    Serial.print(int(totalGyro/gyroThreshold*100));
    Serial.println("% ngưỡng)");
    
    // Hiển thị giá trị tối đa đã đo được
    Serial.print("Giá trị tối đa: Gia tốc = ");
    Serial.print(maxAcceleration, 3);
    Serial.print(" m/s^2, Gyro = ");
    Serial.print(maxGyro, 3);
    Serial.println(" rad/s");
    
    // Hiển thị trạng thái hệ thống
    Serial.print("Trạng thái: ");
    Serial.print(fallDetected ? "Đã phát hiện té ngã, " : "Bình thường, ");
    Serial.println(alarmActive ? "Còi đang BẬT" : "Còi đang TẮT");
    Serial.print("Nút nhấn: ");
    Serial.println(digitalRead(BUTTON_PIN) == LOW ? "ĐANG ĐƯỢC NHẤN" : "KHÔNG ĐƯỢC NHẤN");
  }

  // Phát hiện té ngã
  if (accelerationChange > fallThreshold && totalGyro > gyroThreshold && !fallDetected) {
    Serial.println("\n⚠️⚠️⚠️ TÉ NGÃ ĐƯỢC PHÁT HIỆN! ⚠️⚠️⚠️");
    Serial.print("Biến đổi gia tốc: "); Serial.print(accelerationChange); 
    Serial.print(" m/s^2 (ngưỡng: "); Serial.print(fallThreshold); Serial.println(" m/s^2)");
    Serial.print("Gyro: "); Serial.print(totalGyro); 
    Serial.print(" rad/s (ngưỡng: "); Serial.print(gyroThreshold); Serial.println(" rad/s)");
    
    // Kích hoạt còi báo động
    activateAlarm();
    
    fallDetected = true;
  } 
  else if (accelerationChange < fallThreshold * 0.3 && totalGyro < gyroThreshold * 0.3) {
    // Reset khi cả hai đều giảm đáng kể (30% ngưỡng)
    if (fallDetected) {
      Serial.println("\nTình trạng bình thường trở lại.");
      // Không tắt còi tự động, cần nhấn nút
    }
    fallDetected = false;
    
    // Hiển thị trạng thái bình thường riêng biệt
    if (accelerationChange < 1.0 && totalGyro < 0.5) {
      Serial.println("Tình trạng bình thường.");
    }
  } else {
    // Các trường hợp trung gian
    if (!fallDetected) {
      if (accelerationChange > fallThreshold * 0.7 || totalGyro > gyroThreshold * 0.7) {
        Serial.println("Đang gần tới ngưỡng té ngã...");
      } else if (accelerationChange > fallThreshold * 0.4 || totalGyro > gyroThreshold * 0.4) {
        Serial.println("Có chuyển động đáng chú ý.");
      } else {
        Serial.println("Chuyển động nhẹ.");
      }
    } else {
      Serial.println("Đang theo dõi sau té ngã.");
    }
  }

  // Nút reset giá trị tối đa (qua Serial)
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      maxAcceleration = 0;
      maxGyro = 0;
      Serial.println("\n*** Đã reset giá trị tối đa ***\n");
    }
  }

  Serial.println("------------------------------");
  delay(300);  // Giảm delay xuống 300ms để cải thiện phản hồi với nút nhấn
}

// Hàm kích hoạt còi báo động
void activateAlarm() {
  alarmActive = true;
  alarmStartTime = millis();
  digitalWrite(BUZZER_PIN, HIGH);
  Serial.println("🔊 CÒI BÁO ĐỘNG ĐÃ KÍCH HOẠT!");
}

// Hàm tắt còi báo động
void deactivateAlarm() {
  alarmActive = false;
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("🔇 Còi báo động đã tắt.");
}