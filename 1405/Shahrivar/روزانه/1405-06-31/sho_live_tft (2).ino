#include <SPI.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include "esp_heap_caps.h"

// =====================================================================
//  کتابخونه‌ی Edge Impulse -- قبلش باید از طریق
//  Arduino IDE > Sketch > Include Library > Add .ZIP Library...
//  همون فایل zip خروجی Edge Impulse رو اضافه کرده باشید.
// =====================================================================
#include <Person_Detection_esp_inferencing.h>

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
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

// =====================================================================
//  تشخیص انسان با مدل Edge Impulse (FOMO, ورودی 48x48, بدون نیاز به PSRAM)
// =====================================================================
// به‌جای ساختن یک بافر کامل RGB888 از کل فریم (که فقط با PSRAM جا میشه)،
// مستقیم حین رسیدن هر بلوک JPEG، پیکسل‌های داخل "ناحیه‌ی کراپِ مربعی
// مرکزی" رو در یک گرید 48x48 میانگین‌گیری می‌کنیم. نتیجه دقیقاً معادل
// همون کاری‌ه که crop_and_interpolate_rgb888 انجام می‌داد، ولی بدون
// نیاز به بافر واسط بزرگ.

#define DETECT_EVERY_N_FRAMES 2   // هر چند فریم یک‌بار تشخیص اجرا بشه
#define PERSON_LABEL "person"

// نکته‌ی مهم: آرایه‌ی tensor_arena داخل کتابخونه‌ی EI یک آرایه‌ی static
// (کامپایل‌تایم) هست، یعنی از قبل رزرو شده و اصلاً با heap پویا رقابت
// نمی‌کنه. پس گارد زیر رو فقط به‌عنوان یه چک سلامتی عمومی و کوچیک
// نگه می‌داریم (نه وابسته به اندازه‌ی آرنا)، تا از OOM واقعیِ ناشی از
// انباشته‌شدن بافرهای WebSocket/JPEG جلوگیری کنه؛ نباید مانع اجرای
// عادی مدل بشه.
#define REQUIRED_FREE_HEAP_BYTES 20000

#define MODEL_W EI_CLASSIFIER_INPUT_WIDTH   // 48
#define MODEL_H EI_CLASSIFIER_INPUT_HEIGHT  // 48

// ناحیه‌ی کراپ مرکزیِ مربعی از فریم 480x320 (چون عرض بزرگ‌تره،
// ارتفاع کامل نگه داشته میشه و از چپ‌وراست کراپ میشه):
static const int CROP_SIZE = SCREEN_HEIGHT;                         // 320
static const int CROP_OFFSET_X = (SCREEN_WIDTH - CROP_SIZE) / 2;     // 80
static const int CROP_OFFSET_Y = 0;
static const float MODEL_TO_SCREEN_SCALE = (float)CROP_SIZE / (float)MODEL_W; // 320/48

// آرایه‌های انباشت برای میانگین‌گیری هر سلول از گرید 48x48
static uint32_t accumR[MODEL_H][MODEL_W];
static uint32_t accumG[MODEL_H][MODEL_W];
static uint32_t accumB[MODEL_H][MODEL_W];
static uint16_t accumCount[MODEL_H][MODEL_W];

// بافر نهایی که به کلاسیفایر داده میشه (ترتیب B,G,R تا با
// ei_camera_get_data زیر هم‌خوان باشه)
static uint8_t snapshotBuf[MODEL_W * MODEL_H * 3];

static int frameCounter = 0;
static bool doDetectionThisFrame = false;

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (y >= tft.height()) return 0;

  tft.pushImage(x, y, w, h, bitmap);

  if (doDetectionThisFrame) {
    for (uint16_t j = 0; j < h; j++) {
      int py = y + j;
      if (py < CROP_OFFSET_Y || py >= CROP_OFFSET_Y + CROP_SIZE) continue;
      int ty = (int)((py - CROP_OFFSET_Y) * MODEL_H / CROP_SIZE);
      if (ty < 0 || ty >= MODEL_H) continue;

      uint16_t *srcRow = bitmap + (j * w);
      for (uint16_t i = 0; i < w; i++) {
        int px = x + i;
        if (px < CROP_OFFSET_X || px >= CROP_OFFSET_X + CROP_SIZE) continue;
        int tx = (int)((px - CROP_OFFSET_X) * MODEL_W / CROP_SIZE);
        if (tx < 0 || tx >= MODEL_W) continue;

        uint16_t pxv = srcRow[i];
        uint8_t r5 = (pxv >> 11) & 0x1F;
        uint8_t g6 = (pxv >> 5) & 0x3F;
        uint8_t b5 = pxv & 0x1F;
        uint8_t r8 = (r5 * 527 + 23) >> 6;
        uint8_t g8 = (g6 * 259 + 33) >> 6;
        uint8_t b8 = (b5 * 527 + 23) >> 6;

        accumR[ty][tx] += r8;
        accumG[ty][tx] += g8;
        accumB[ty][tx] += b8;
        accumCount[ty][tx]++;
      }
    }
  }

  return 1;
}

// این تابع رو کتابخونه‌ی Edge Impulse صدا می‌زنه تا داده‌ی پیکسل
// بافر 48x48 (snapshotBuf) رو به‌صورت RGB بسته‌بندی‌شده (24 بیتی) بخونه.
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (snapshotBuf[pixel_ix + 2] << 16) +
                          (snapshotBuf[pixel_ix + 1] << 8) +
                           snapshotBuf[pixel_ix];
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}

void runPersonDetection() {
  // --- محافظ حافظه: قبل از اجرای مدل، مطمئن شو یه بلوک پیوسته‌ی به‌اندازه‌ی
  //     کافی آزاده. اگه نبود، به‌جای کرش، فقط این فریم رو رد کن. ---
  size_t largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  if (largestFreeBlock < REQUIRED_FREE_HEAP_BYTES) {
    Serial.printf("[EI] SKIP: heap critically low (largest free block=%u bytes). "
                  "Skipping this frame to avoid a crash.\n",
                  (unsigned)largestFreeBlock);
    return;
  }

  // مرحله ۱: از آرایه‌های انباشت، بافر نهایی 48x48 (B,G,R) رو بساز
  for (int ty = 0; ty < MODEL_H; ty++) {
    for (int tx = 0; tx < MODEL_W; tx++) {
      size_t idx = (ty * MODEL_W + tx) * 3;
      uint16_t cnt = accumCount[ty][tx];
      if (cnt == 0) {
        // سلولی که هیچ پیکسلی بهش نگاشت نشده (نادر) -> خاکستری خنثی
        snapshotBuf[idx + 0] = 128;
        snapshotBuf[idx + 1] = 128;
        snapshotBuf[idx + 2] = 128;
      } else {
        snapshotBuf[idx + 0] = (uint8_t)(accumB[ty][tx] / cnt); // B
        snapshotBuf[idx + 1] = (uint8_t)(accumG[ty][tx] / cnt); // G
        snapshotBuf[idx + 2] = (uint8_t)(accumR[ty][tx] / cnt); // R
      }
    }
  }

  // مرحله ۲: اجرای مدل
  ei::signal_t signal;
  signal.total_length = MODEL_W * MODEL_H;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    Serial.printf("[EI] run_classifier failed: %d\n", err);
    return;
  }

  tft.fillRect(0, 0, SCREEN_WIDTH, 22, TFT_BLACK);

  bool personFound = false;
  float bestConfidence = 0;

  for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    if (bb.value == 0) continue;
    if (strcmp(bb.label, PERSON_LABEL) != 0) continue;

    personFound = true;
    if (bb.value > bestConfidence) bestConfidence = bb.value;

    int screenX = CROP_OFFSET_X + (int)(bb.x * MODEL_TO_SCREEN_SCALE);
    int screenY = CROP_OFFSET_Y + (int)(bb.y * MODEL_TO_SCREEN_SCALE);
    int screenW = (int)(bb.width * MODEL_TO_SCREEN_SCALE);
    int screenH = (int)(bb.height * MODEL_TO_SCREEN_SCALE);

    tft.drawRect(screenX, screenY, screenW, screenH, TFT_RED);
    tft.drawRect(screenX - 1, screenY - 1, screenW + 2, screenH + 2, TFT_RED);

    Serial.printf("[EI] person (%.2f) model(x=%d y=%d w=%d h=%d) -> screen(x=%d y=%d w=%d h=%d)\n",
                  bb.value, bb.x, bb.y, bb.width, bb.height, screenX, screenY, screenW, screenH);
  }

  tft.setTextColor(personFound ? TFT_RED : TFT_GREEN, TFT_BLACK);
  tft.setCursor(4, 2);
  if (personFound) {
    tft.print("PERSON DETECTED  ");
    tft.print((int)(bestConfidence * 100));
    tft.print("%");
  } else {
    tft.print("No person");
  }

  Serial.printf("[EI] timing: dsp=%d ms, classify=%d ms\n",
                result.timing.dsp, result.timing.classification);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n-------------Create ESP32-S3 as Access Point");
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  WiFi.softAP(ssid, password, 6);
  delay(500);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  Serial.print("\nAP IP Address : ");
  Serial.println(WiFi.softAPIP());

  server.listen(8888);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLUE);

  centerX = SCREEN_WIDTH / 2;
  centerY = SCREEN_HEIGHT / 2;

  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.drawCentreString("Waiting for connection", centerX, centerY - 15, FONT_SIZE);
  tft.drawCentreString("from ESP32-CAM...", centerX, centerY + 5, FONT_SIZE);

  tft.setSwapBytes(true);
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);

  Serial.println("Setup complete (no PSRAM required in this version).");
  Serial.printf("[DIAG] Free heap: %u bytes | Largest free block: %u bytes | Static tensor_arena reserved: %u bytes\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                (unsigned)EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE);
}

void loop() {
  if (server.poll()) {
    client = server.accept();
    tft.fillScreen(TFT_BLACK);
    frameCounter = 0;
  }

  if (client.available()) {
    client.poll();
    WebsocketsMessage msg = client.readBlocking();
    if (msg.length() > 0) {
      frameCounter++;
      doDetectionThisFrame = (frameCounter >= DETECT_EVERY_N_FRAMES);

      if (doDetectionThisFrame) {
        memset(accumR, 0, sizeof(accumR));
        memset(accumG, 0, sizeof(accumG));
        memset(accumB, 0, sizeof(accumB));
        memset(accumCount, 0, sizeof(accumCount));
      }

      TJpgDec.drawJpg(0, 0, (const uint8_t *)msg.c_str(), msg.length());

      if (doDetectionThisFrame) {
        frameCounter = 0;
        runPersonDetection();
      }
    }
  }
}

// =====================================================================
//  نکات مهم نصب و کامپایل
// =====================================================================
// 1. کتابخونه: Sketch > Include Library > Add .ZIP Library... و همون
//    zip خروجی Edge Impulse رو انتخاب کنید.
// 2. این نسخه دیگر به PSRAM نیاز ندارد (Tools > PSRAM می‌تواند
//    "Disabled" هم باشد)، چون همه‌ی بافرها کوچک و در RAM داخلی هستند.
// 3. Tools > Partition Scheme: همچنان "Huge APP" را نگه دارید، چون
//    خودِ کتابخونه‌ی مدل حجیم است (این محدودیت به فلش مربوط است نه PSRAM).
// 4. DETECT_EVERY_N_FRAMES: اگر تصویر کند/لگ‌دار شد، این عدد را
//    بزرگ‌تر کنید (مثلاً 3 یا 4).
// =====================================================================
