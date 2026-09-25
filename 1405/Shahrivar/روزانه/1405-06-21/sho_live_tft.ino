// #include <SPI.h>
// #include <TFT_eSPI.h>
// #include <TJpg_Decoder.h>
// #include <ArduinoWebsockets.h>
// #include <WiFi.h>

// #define SCREEN_WIDTH 480
// #define SCREEN_HEIGHT 320
// #define FONT_SIZE 2

// const char *ssid = "ESP32CAM_to_ESP32"; //--> access point name.
// const char *password = "myesp32server"; //--> access point password.

// IPAddress local_ip(192, 168, 1, 1);
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 255, 0);

// int centerX, centerY;

// using namespace websockets;
// WebsocketsServer server;
// WebsocketsClient client;

// TFT_eSPI tft = TFT_eSPI();

// bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
// {
//   // Stop further decoding as image is running off bottom of screen.
//   if (y >= tft.height())
//     return 0;

//   // This function will clip the image block rendering automatically at the TFT boundaries.
//   tft.pushImage(x, y, w, h, bitmap);

//   // This might work instead if you adapt the sketch to use the Adafruit_GFX library.
//   // tft.drawRGBBitmap(x, y, bitmap, w, h);

//   // Return 1 to decode next block.
//   return 1;
// }

// void setup()
// {

//   Serial.begin(115200);
//   Serial.println();
//   delay(3000);

//   Serial.println();
//   Serial.println("-------------Create ESP32 as Access Point and start the server.");
//   Serial.println("WIFI mode : AP");
//   WiFi.mode(WIFI_AP);

//   Serial.println();
//   Serial.println("Setting AP.");
//   WiFi.softAP(ssid, password);
//   delay(500);
//   WiFi.softAPConfig(local_ip, gateway, subnet);

//   IPAddress IP = WiFi.softAPIP();
//   Serial.println();
//   Serial.print("AP IP Address : ");
//   Serial.println(IP);

//   server.listen(8888);
//   Serial.println();
//   Serial.print("Is server live ? ");
//   Serial.println(server.available());
//   Serial.println("-------------");

//   tft.begin();
//   tft.setRotation(1);
//   tft.fillScreen(TFT_BLUE);

//   centerX = SCREEN_WIDTH / 2;
//   centerY = SCREEN_HEIGHT / 2;

//   tft.setTextColor(TFT_WHITE, TFT_BLUE);
//   tft.drawCentreString("Waiting for connection", centerX, centerY - 15, FONT_SIZE);
//   tft.drawCentreString("from ESP32-CAM (Client)", centerX, centerY + 5, FONT_SIZE);
//   Serial.println();
//   Serial.println("Waiting for connection from ESP32-CAM (Client).");

//   tft.setSwapBytes(true);

//   TJpgDec.setJpgScale(1);

//   TJpgDec.setCallback(tft_output);
// }

// void loop()
// {
//   if (server.poll())
//   {
//     client = server.accept();
//   }

//   if (client.available())
//   {
//     client.poll();

//     WebsocketsMessage msg = client.readBlocking();

//     uint32_t t = millis();

//     uint16_t w = 0, h = 0;
//     TJpgDec.getJpgSize(&w, &h, (const uint8_t *)msg.c_str(), msg.length());
//     Serial.print("Width = ");
//     Serial.print(w);
//     Serial.print(", height = ");
//     Serial.println(h);
//     TJpgDec.drawJpg(0, 0, (const uint8_t *)msg.c_str(), msg.length());
//     t = millis() - t;
//     Serial.print(t);
//     Serial.println(" ms");
//   }
// }
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>

// #define SCREEN_WIDTH 480
// #define SCREEN_HEIGHT 320


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

#define FONT_SIZE 2

const char *ssid = "ESP32CAM_to_ESP32";
const char *password = "myesp32server";

IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

int centerX, centerY;

using namespace websockets;
WebsocketsServer server;
WebsocketsClient client;

TFT_eSPI tft = TFT_eSPI();

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (y >= tft.height()) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n-------------Create ESP32 as Access Point");
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false); // غیرفعال کردن ذخیره انرژی برای کاهش لگ وای‌فای

  // تنظیم کانال وای فای روی 6 برای کاهش تداخل
  WiFi.softAP(ssid, password, 6); 
  delay(500);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  Serial.print("\nAP IP Address : ");
  Serial.println(WiFi.softAPIP());

  server.listen(8888);

  tft.begin();
  tft.setRotation(1); // در صورت نیاز بین 1 یا 3 تغییر دهید
  tft.fillScreen(TFT_BLUE);

  centerX = SCREEN_WIDTH / 2;
  centerY = SCREEN_HEIGHT / 2;

  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.drawCentreString("Waiting for connection", centerX, centerY - 15, FONT_SIZE);
  tft.drawCentreString("from ESP32-CAM...", centerX, centerY + 5, FONT_SIZE);

  tft.setSwapBytes(true);
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);
}

void loop() {
  if (server.poll()) {
    client = server.accept();
    tft.fillScreen(TFT_BLACK); // پاک کردن صفحه هنگام اتصال کلاینت
  }

  if (client.available()) {
    client.poll();
    WebsocketsMessage msg = client.readBlocking();
    if (msg.length() > 0) {
      TJpgDec.drawJpg(0, 0, (const uint8_t *)msg.c_str(), msg.length());
    }
  }
}
