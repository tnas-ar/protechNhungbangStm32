#define BLYNK_TEMPLATE_ID "TMPL67_8aaz2E"
#define BLYNK_TEMPLATE_NAME "STM32 LM75"
#define BLYNK_AUTH_TOKEN  "9tH7omju3PVSkPNN6NfTmMWbmG1pbrdM"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// WIFI (Luu y: Dien thoai va may tinh phai dung chung mot mang wifi)
char ssid[] = "/chinori";       // 🔹 Thay bang ten wifi cua may
char pass[] = "typphono";   // 🔹 Thay bang pass wifi cua may

BlynkTimer timer;

void setup() {
  Serial.begin(115200);                        // Debug Monitor
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // STM32 connected here (TX->RX2, RX->TX2)
  Serial.println("ESP32 Blynk + UART Ready...");

  // Connect to Blynk Cloud
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();

  // Check if STM32 sent data
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    float temp = line.toFloat();

    if (temp > -200 && temp < 200) {   // sanity check
      Serial.print("Got temp from STM32: ");
      Serial.println(temp);

      // Send to Blynk app (Virtual Pin V0)
      Blynk.virtualWrite(V0, temp);
    }
  }
}
