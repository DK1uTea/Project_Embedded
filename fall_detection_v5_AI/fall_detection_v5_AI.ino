// First define your Blynk configuration
#define BLYNK_TEMPLATE_ID "TMPL6GPaA0o6d"
#define BLYNK_TEMPLATE_NAME "Fall Detection"
#define BLYNK_AUTH_TOKEN "DxmxA1ZfX5Z-xiiqiN2etguKkYRx7zdi"
#define BLYNK_PRINT Serial             // In log Blynk qua Serial

// Then include the libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>  // Include Blynk library after defining template ID and name
#include "fall_detection_model.h" // Include the AI model header

// ==== PINS CONFIGURATION ====
#define BUZZER_PIN 14        // Chân còi kết nối với cổng 14
#define BUTTON_PIN 12        // Chân nút nhấn kết nối với cổng 12
#define LED_PIN 2           // LED để hiển thị trạng thái

// ==== BLYNK CONFIGURATION ====
char auth[] = BLYNK_AUTH_TOKEN;        // Blynk Auth Token
char ssid[] = "Asus";                  // Tên WiFi của bạn
char pass[] = "trangckun";             // Mật khẩu WiFi

// Biến theo dõi kết nối
bool isWifiConnected = false;
bool isBlynkConnected = false;
unsigned long lastConnectionAttempt = 0;
const unsigned long reconnectInterval = 30000; // 30 giây giữa các lần thử kết nối lại

// Virtual pin để gửi thông báo
#define VPIN_FALL_DETECTION V0
#define VPIN_SYSTEM_STATUS V1
#define VPIN_ACCEL_TOTAL V3
#define VPIN_GYRO_TOTAL  V4
#define VPIN_FALL_DETAIL V5
#define VPIN_ACTIVITY_TYPE V6  // Thêm virtual pin để hiển thị loại hoạt động

// ==== MPU6050 CONFIGURATION ====
Adafruit_MPU6050 mpu;

// Ngưỡng phát hiện té ngã
const float GRAVITY = 9.81;           // Gia tốc trọng trường

// ==== AI MODEL CONFIGURATION ====
#define WINDOW_SIZE 50                // Kích thước cửa sổ dữ liệu cho AI
float accel_x_buffer[WINDOW_SIZE];    // Buffer lưu dữ liệu trục X
float accel_y_buffer[WINDOW_SIZE];    // Buffer lưu dữ liệu trục Y
float accel_z_buffer[WINDOW_SIZE];    // Buffer lưu dữ liệu trục Z
int buffer_index = 0;                 // Chỉ số hiện tại trong buffer
bool buffer_filled = false;           // Buffer đã đầy chưa
float features[18];                   // Mảng lưu các đặc trưng tính toán được

// Mảng tên các loại hoạt động
const char* activity_names[] = {
  "Ngồi xuống",   // downSit (0)
  "Ngã tự do",    // freeFall (1)
  "Ngã khi chạy", // runFall (2)
  "Chạy và ngồi", // runSit (3)
  "Ngã khi đi bộ", // walkFall (4)
  "Đi bộ và ngồi" // walkSit (5)
};

// Mảng các loại té ngã (dựa trên mô hình của bạn)
const int fall_activity_ids[] = {1, 2}; // freeFall, runFall, walkFall

// Biến để lưu giá trị tối đa cho mục đích kiểm tra
float maxAcceleration = 0;
float maxGyro = 0;

// Biến trạng thái
bool fallDetected = false;        // Biến cờ để theo dõi té ngã
unsigned long fallDetectedTime = 0;
const unsigned long resetTimeout = 60000;
bool alarmActive = false;         // Biến theo dõi trạng thái còi báo động
bool testMode = true;             // Chế độ kiểm tra - hiển thị thông tin debug

// Biến xử lý còi
unsigned long alarmStartTime = 0;
const unsigned long alarmDuration = 10000;  // Thời gian còi kêu tối đa (10 giây)

// Biến xử lý nút nhấn
int lastButtonState = HIGH;       // Trạng thái trước đó của nút
unsigned long lastButtonPressTime = 0; // Thời điểm nhấn nút cuối cùng

// Blynk connected callback
BLYNK_CONNECTED() {
  isBlynkConnected = true;
  Serial.println("✅ BLYNK ĐÃ KẾT NỐI THÀNH CÔNG!");
  
  // Đồng bộ trạng thái với app khi kết nối lại
  Blynk.virtualWrite(VPIN_SYSTEM_STATUS, "Thiết bị trực tuyến");
  
  // Nếu đang có cảnh báo té ngã, gửi lại thông báo
  if (fallDetected) {
    Blynk.virtualWrite(VPIN_FALL_DETECTION, "⚠️ TÉ NGÃ ĐƯỢC PHÁT HIỆN!");
    Blynk.logEvent("fall", "Phát hiện té ngã lúc " + String(millis()/1000) + "s kể từ khi khởi động");
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n------------------------------------");
  Serial.println("🔄 KHỞI ĐỘNG HỆ THỐNG PHÁT HIỆN TÉ NGÃ - MÔ HÌNH AI");
  Serial.println("------------------------------------");

  // Cài đặt chân GPIO
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  Serial.println("✓ Đã cài đặt GPIO: Còi (14), Nút nhấn (12), LED (2)");

  Wire.begin(2, 15);
  Serial.println("✓ I2C được khởi tạo trên SDA=2, SCL=15");

  // Khởi tạo MPU6050
  Serial.print("⏳ Đang khởi tạo MPU6050... ");
  if (!mpu.begin()) {
    Serial.println("❌ KHÔNG TÌM THẤY MPU6050!");
    Serial.println("   Kiểm tra kết nối phần cứng.");
    while (1) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(300);
      digitalWrite(BUZZER_PIN, LOW);
      delay(300);
    }
  }
  Serial.println("✅ MPU6050 đã khởi động thành công!");
  
  // Cài đặt phạm vi đo cho MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("✓ Phạm vi đo gia tốc kế: ±8g");

  // Khởi tạo buffer
  for (int i = 0; i < WINDOW_SIZE; i++) {
    accel_x_buffer[i] = 0;
    accel_y_buffer[i] = 0;
    accel_z_buffer[i] = 0;
  }
  Serial.println("✓ Khởi tạo buffer dữ liệu cho AI");
  
  // Kết nối WiFi và Blynk
  connectToNetwork();
  
  // Hiển thị thông tin cấu hình
  Serial.println("\n--- CẤU HÌNH HỆ THỐNG ---");
  Serial.println("• WiFi: " + String(ssid));
  Serial.println("• Blynk Template ID: " + String(BLYNK_TEMPLATE_ID));
  Serial.println("• Blynk Template Name: " + String(BLYNK_TEMPLATE_NAME));
  Serial.println("• Mô hình AI: Random Forest");
  Serial.println("• Kích thước buffer: " + String(WINDOW_SIZE) + " mẫu");
  Serial.println("• Loại té ngã phát hiện: freeFall, runFall, walkFall");
  Serial.println("------------------------------------");

  // Đợi buffer đầy trước khi bắt đầu phát hiện
  Serial.println("⏳ Đang chờ thu thập dữ liệu gia tốc...");
}

void loop() {
  // Chớp LED để biết hệ thống đang hoạt động
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = millis();
  }

  // Kiểm tra và duy trì kết nối
  checkConnection();
  
  // Chỉ chạy Blynk.run() khi có kết nối WiFi
  if (isWifiConnected) {
    Blynk.run();
  }

  // Đọc trạng thái nút nhấn
  int buttonState = digitalRead(BUTTON_PIN);
  
  // Xử lý khi nút thay đổi trạng thái
  if (buttonState != lastButtonState) {
    if (testMode) {
      Serial.println("Trạng thái nút: " + String(buttonState == LOW ? "NHẤN" : "NHẢ"));
    }
    lastButtonState = buttonState;
    
    // Khi nút được nhấn (LOW với INPUT_PULLUP)
    if (buttonState == LOW) {
      lastButtonPressTime = millis();
      
      // Tắt còi nếu đang kích hoạt
      if (alarmActive) {
        Serial.println("NÚT NHẤN ĐƯỢC PHÁT HIỆN - TẮT CÒI!");
        deactivateAlarm();
      }
    }
  }
  
  // Quản lý còi báo động
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

  // Cập nhật giá trị trên Blynk nếu kết nối
  if (isBlynkConnected) {
    static unsigned long lastBlynkUpdate = 0;
    if (millis() - lastBlynkUpdate > 1000) { // Cập nhật mỗi giây
      Blynk.virtualWrite(VPIN_ACCEL_TOTAL, totalAcceleration);
      Blynk.virtualWrite(VPIN_GYRO_TOTAL, totalGyro);
      lastBlynkUpdate = millis();
    }
  }

  // Cập nhật giá trị tối đa
  if (totalAcceleration > maxAcceleration) maxAcceleration = totalAcceleration;
  if (totalGyro > maxGyro) maxGyro = totalGyro;

  // =====================================================
  // PHẦN AI: Lưu dữ liệu vào buffer và phát hiện té ngã
  // =====================================================
  
  // Lưu giá trị gia tốc vào buffer
  accel_x_buffer[buffer_index] = accel.acceleration.x;
  accel_y_buffer[buffer_index] = accel.acceleration.y;
  accel_z_buffer[buffer_index] = accel.acceleration.z;
  
  // Cập nhật chỉ số buffer
  buffer_index = (buffer_index + 1) % WINDOW_SIZE;
  
  // Kiểm tra buffer đã đầy chưa
  if (buffer_index == 0 && !buffer_filled) {
    buffer_filled = true;
    Serial.println("✅ Buffer đã đầy, bắt đầu phát hiện té ngã bằng AI");
  }
  
  // Chỉ xử lý khi buffer đã đầy
  if (buffer_filled) {
    // Trích xuất đặc trưng từ dữ liệu trong buffer
    extract_features(accel_x_buffer, accel_y_buffer, accel_z_buffer, WINDOW_SIZE, features);
    
    // Phát hiện loại hoạt động
    int activity = detect_activity(features);
    
    // Kiểm tra xem loại hoạt động có phải té ngã không
    bool is_fall = false;
    for (int i = 0; i < sizeof(fall_activity_ids)/sizeof(fall_activity_ids[0]); i++) {
      if (activity == fall_activity_ids[i]) {
        is_fall = true;
        break;
      }
    }
    
    // Báo cáo kết quả phát hiện
    static int last_activity = -1;
    
    // Chỉ log khi hoạt động thay đổi để tránh log quá nhiều
    if (activity != last_activity) {
      String activityName = (activity >= 0 && activity < 6) ? 
                           String(activity_names[activity]) : "Không xác định";
      
      Serial.println("AI đã phát hiện hoạt động: " + activityName);
      
      // Cập nhật lên Blynk nếu đã kết nối
      if (isBlynkConnected) {
        Blynk.virtualWrite(VPIN_ACTIVITY_TYPE, activityName);
      }
      
      last_activity = activity;
    }
    
    // Xử lý phát hiện té ngã
    if (is_fall && !fallDetected) {
      String fallType = (activity >= 0 && activity < 6) ? 
                       String(activity_names[activity]) : "Không xác định";
      
      Serial.println("\n⚠️⚠️⚠️ TÉ NGÃ ĐƯỢC PHÁT HIỆN (" + fallType + ")! ⚠️⚠️⚠️");
      Serial.println("Loại té ngã: " + fallType);
      Serial.println("Trình phân loại AI: ID " + String(activity));
      
      // Kích hoạt còi báo động
      activateAlarm();
      
      fallDetected = true;
      fallDetectedTime = millis(); // Ghi nhớ thời điểm phát hiện té ngã

      // Gửi thông báo đến Blynk nếu đã kết nối
      if (isBlynkConnected) {
        Serial.println("📲 Đang gửi thông báo té ngã qua Blynk...");

        // Tạo ID ngẫu nhiên và thông tin thời gian để tạo sự khác biệt
        String eventID = String(random(10000, 99999));
        unsigned long currentTime = millis() / 1000;
        String timestamp = String(currentTime);
        
        // Tạo thông báo chi tiết
        String eventMsg = "⚠️ TÉ NGÃ #" + eventID + " | " + timestamp + "\n" +
                         "• Loại té ngã: " + fallType + "\n" +
                         "• Gia tốc: " + String(totalAcceleration, 1) + " m/s²\n" +
                         "• Gyro: " + String(totalGyro, 1) + " rad/s";

        Blynk.virtualWrite(VPIN_FALL_DETECTION, "⚠️ TÉ NGÃ ĐƯỢC PHÁT HIỆN!");
        Blynk.logEvent("fall", eventMsg);
        Serial.println("✅ Đã gửi thông báo đến Blynk: " + eventMsg);

        // --- Ghi nhận chi tiết vào Terminal/Label widget ---
        String detailMsg = 
          "Té ngã #" + eventID + " [" + timestamp + "s]\n"
          "Loại: " + fallType + "\n"
          "Accel: X=" + String(accel.acceleration.x,2) +
          " Y=" + String(accel.acceleration.y,2) +
          " Z=" + String(accel.acceleration.z,2) + "\n"
          "Gyro: X=" + String(gyro.gyro.x,2) +
          " Y=" + String(gyro.gyro.y,2) +
          " Z=" + String(gyro.gyro.z,2);

        Blynk.virtualWrite(VPIN_FALL_DETAIL, detailMsg); // V5, dùng cho Terminal/Label widget
        Serial.println("✅ Đã ghi nhật ký chi tiết té ngã: " + detailMsg);
      } else {
        Serial.println("❌ Không thể gửi thông báo (Blynk chưa kết nối)");
      }
    } else if (!is_fall && fallDetected) {
      // Reset trạng thái nếu không còn phát hiện té ngã
      Serial.println("\nTình trạng bình thường trở lại.");
      fallDetected = false;
      
      // Cập nhật trạng thái trên Blynk nếu đã kết nối
      if (isBlynkConnected) {
        Blynk.virtualWrite(VPIN_FALL_DETECTION, "Bình thường");
      }
    }
  }

  // Tự động reset trạng thái sau một thời gian
  if (fallDetected && (millis() - fallDetectedTime > resetTimeout)) {
    Serial.println("⏱️ Tự động reset trạng thái té ngã sau 60 giây");
    fallDetected = false;
    if (isBlynkConnected) {
      Blynk.virtualWrite(VPIN_FALL_DETECTION, "Trạng thái bình thường");
    }
  }

  // In ra thông tin debug trong chế độ kiểm tra
  if (testMode) {
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 2000) { // In debug mỗi 2 giây
      // In chi tiết gia tốc từng trục
      Serial.print("Accel (m/s^2) -> X: "); Serial.print(accel.acceleration.x, 3);
      Serial.print(" Y: "); Serial.print(accel.acceleration.y, 3);
      Serial.print(" Z: "); Serial.println(accel.acceleration.z, 3);

      // In chi tiết gyro từng trục
      Serial.print("Gyro (rad/s) -> X: "); Serial.print(gyro.gyro.x, 3);
      Serial.print(" Y: "); Serial.print(gyro.gyro.y, 3);
      Serial.print(" Z: "); Serial.println(gyro.gyro.z, 3);

      // Hiển thị trạng thái hệ thống
      Serial.print("Trạng thái: ");
      Serial.print(fallDetected ? "Đã phát hiện té ngã, " : "Bình thường, ");
      Serial.print(alarmActive ? "Còi BẬT, " : "Còi TẮT, ");
      Serial.print(buffer_filled ? "Buffer đầy, " : "Buffer chưa đầy, ");
      Serial.println(isBlynkConnected ? "Blynk kết nối OK" : "Blynk KHÔNG kết nối");
      
      Serial.println("------------------------------");
      lastDebugPrint = millis();
    }
  }

  delay(20); // Giảm thời gian delay để có thể lấy mẫu nhanh hơn
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

// Hàm kết nối WiFi và Blynk với thêm nhiều delay để quan sát log
void connectToNetwork() {
  Serial.println("\n------------------------------------");
  Serial.println("⏳ BẮT ĐẦU QUÁ TRÌNH KẾT NỐI MẠNG");
  Serial.println("------------------------------------");
  delay(1000); // Delay 1 giây để quan sát log
  
  // === GIAI ĐOẠN 1: CHUẨN BỊ KẾT NỐI WIFI ===
  Serial.println("GIAI ĐOẠN 1: Chuẩn bị kết nối WiFi");
  Serial.println("• SSID: " + String(ssid));
  Serial.println("• Mật khẩu: ********");
  Serial.print("• Đang chuẩn bị... ");
  
  for (int i = 3; i > 0; i--) {
    Serial.print(String(i) + "... ");
    delay(1000); // Đếm ngược 3 giây
  }
  Serial.println("BẮT ĐẦU!");
  
  // === GIAI ĐOẠN 2: KẾT NỐI WIFI ===
  Serial.println("\nGIAI ĐOẠN 2: Đang kết nối WiFi");
  WiFi.begin(ssid, pass);
  
  // Đợi kết nối WiFi với đếm thời gian
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    if (attempts % 10 == 9) Serial.println(); // Xuống dòng sau mỗi 10 dấu chấm
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    isWifiConnected = true;
    Serial.println("\n✅ WIFI ĐÃ KẾT NỐI THÀNH CÔNG!");
    Serial.println("• Địa chỉ IP: " + WiFi.localIP().toString());
    Serial.println("• Cường độ tín hiệu: " + String(WiFi.RSSI()) + " dBm");
    Serial.println("• MAC Address: " + WiFi.macAddress());
    
    // Delay để quan sát thông tin WiFi
    Serial.println("• Đợi 3 giây trước khi kết nối Blynk...");
    for (int i = 3; i > 0; i--) {
      Serial.print(String(i) + "... ");
      delay(1000);
    }
    Serial.println("Tiếp tục!");
    
    // === GIAI ĐOẠN 3: KẾT NỐI BLYNK ===
    Serial.println("\nGIAI ĐOẠN 3: Đang kết nối Blynk");
    Serial.println("• Auth Token: " + String(auth));
    Serial.println("• Server mặc định: blynk.cloud");
    Serial.print("• Đang kết nối");
    
    Blynk.config(auth);
    
    // Đếm ngược với dấu chấm trong khi kết nối
    unsigned long startTime = millis();
    while (!Blynk.connected() && (millis() - startTime < 10000)) {
      Serial.print(".");
      delay(500);
    }
    
    if (Blynk.connected()) {
      isBlynkConnected = true;
      Serial.println("\n✅ BLYNK ĐÃ KẾT NỐI THÀNH CÔNG!");
      Serial.println("• Template ID: " + String(BLYNK_TEMPLATE_ID));
      Serial.println("• Template Name: " + String(BLYNK_TEMPLATE_NAME));
      
      // Cập nhật trạng thái thiết bị lên Blynk
      Blynk.virtualWrite(VPIN_SYSTEM_STATUS, "Thiết bị trực tuyến");
      Serial.println("• Đã cập nhật trạng thái lên Blynk Cloud");
      
      // Đợi thêm 2 giây để quan sát
      delay(2000);
    } else {
      Serial.println("\n❌ BLYNK KẾT NỐI KHÔNG THÀNH CÔNG!");
      Serial.println("• Lỗi: Không thể kết nối đến máy chủ Blynk");
      Serial.println("• Hệ thống sẽ tiếp tục vận hành ở chế độ ngoại tuyến");
      delay(2000); // Đợi 2 giây để quan sát thông báo lỗi
    }
  } else {
    Serial.println("\n❌ WIFI KẾT NỐI KHÔNG THÀNH CÔNG!");
    Serial.println("• Lỗi: Không thể kết nối đến mạng WiFi");
    Serial.println("• Vui lòng kiểm tra SSID và mật khẩu");
    Serial.println("• Hệ thống sẽ tiếp tục vận hành ở chế độ ngoại tuyến");
    delay(3000); // Đợi 3 giây để quan sát thông báo lỗi
  }
  
  Serial.println("\n------------------------------------");
  Serial.println("QUÁ TRÌNH KẾT NỐI HOÀN TẤT");
  Serial.println("------------------------------------");
  delay(1000); // Delay cuối cùng
  
  lastConnectionAttempt = millis();
}

// Hàm kiểm tra và duy trì kết nối
void checkConnection() {
  // Kiểm tra trạng thái WiFi
  if (WiFi.status() != WL_CONNECTED) {
    if (isWifiConnected) {
      isWifiConnected = false;
      isBlynkConnected = false;
      Serial.println("❌ WiFi bị ngắt kết nối!");
    }
    
    // Thử kết nối lại sau một khoảng thời gian
    if (millis() - lastConnectionAttempt > reconnectInterval) {
      Serial.println("🔄 Đang thử kết nối lại...");
      connectToNetwork();
    }
  }
  
  // Kiểm tra kết nối Blynk (nếu WiFi đã kết nối)
  if (isWifiConnected && !isBlynkConnected) {
    // Thử kết nối lại Blynk nếu đã đủ thời gian
    if (millis() - lastConnectionAttempt > reconnectInterval) {
      Serial.print("🔄 Thử kết nối lại Blynk... ");
      if (Blynk.connect(5000)) {
        isBlynkConnected = true;
        Serial.println("✅ Đã kết nối lại!");
        Blynk.virtualWrite(VPIN_SYSTEM_STATUS, "Thiết bị trực tuyến");
      } else {
        Serial.println("❌ Không thành công");
      }
      lastConnectionAttempt = millis();
    }
  }
}