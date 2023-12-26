#include <TFT_eSPI.h>
#include <driver/i2s.h>

#define ADC_INPUT (ADC1_CHANNEL_4)  //pin 32
#define I2S_DMA_BUF_LEN (50000)

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
uint16_t trigger_val = 0;
uint16_t buf[I2S_DMA_BUF_LEN];

uint16_t pixel_data[120];
uint16_t pixel_data_tmp[120];

byte auto_trig = 1;
byte s_div_idx = 0;
byte v_div_idx = 1;
byte hold = 0;
byte data_ready = 0;
byte drawing = 0;
byte menu = 0;

uint32_t buttonStartTime;
byte buttonDirection;
uint32_t prevChangeTime;

TaskHandle_t task_menu;
TaskHandle_t task_adc;

uint16_t time_divisions[9] = { 10, 25, 50, 100, 250, 500, 1000, 2500, 5000 };
float voltage_divisions[3] = { 0.5, 1, 2 };

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  sprite.createSprite(160, 128);
  tft.fillScreen(TFT_BLACK);
  analogWriteFrequency(500);
  analogWrite(12, 125);

  pinMode(19, INPUT_PULLDOWN);
  pinMode(21, INPUT_PULLDOWN);
  pinMode(17, INPUT_PULLDOWN);

  configure_i2s(1000000);

  xTaskCreatePinnedToCore(
    core0_task,
    "menu",
    10000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    0,          /* Priority of the task */
    &task_menu, /* Task handle. */
    0);         /* Core where the task should run */

  xTaskCreatePinnedToCore(
    core1_task,
    "sampling",
    10000,     /* Stack size in words */
    NULL,      /* Task input parameter */
    3,         /* Priority of the task */
    &task_adc, /* Task handle. */
    1);        /* Core where the task should run */

  attachInterrupt(19, up_handler, RISING);
  attachInterrupt(21, menu_handler, RISING);
  attachInterrupt(17, down_handler, RISING);
}
void up_handler() {
  buttonStartTime = millis();
  buttonDirection = 0;
  up();
}

void up() {
  switch (menu) {
    case 0:
      s_div_idx = (s_div_idx + 1) % 9;
      break;
    case 1:
      v_div_idx = (v_div_idx + 1) % 3;
      break;
    case 2:
      hold = !hold;
      break;
    case 3:
      auto_trig = !auto_trig;
      break;
    case 4:
      trigger_val = (trigger_val + 10) % 4095;
      break;
  }
}
void menu_handler() {
  menu = (menu + 1) % 5;
}

void down_handler() {
  buttonStartTime = millis();
  buttonDirection = 1;
  down();
}
void down() {
  switch (menu) {
    case 0:
      s_div_idx = (s_div_idx - 1 + 9) % 9;
      break;
    case 1:
      v_div_idx = (v_div_idx - 1 + 3) % 3;
      break;
    case 2:
      hold = !hold;
      break;
    case 3:
      auto_trig = !auto_trig;
      break;
    case 4:
      trigger_val = (trigger_val - 10 + 4095) % 4095;
      break;
  }
}

void core0_task(void *pvParameters) {

  (void)pvParameters;

  while (1) {
    if (hold) continue;
    if (drawing) continue;
    data_ready = 0;

    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, buf, 50000 * sizeof(uint16_t), &bytes_read, portMAX_DELAY);
    if (auto_trig) {
      trigger_val = auto_trigger(buf, 50000);
    }
    process_buf(buf, pixel_data);

    data_ready = 1;
    yield();
    // vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void core1_task(void *pvParameters) {

  (void)pvParameters;

  while (1) {
    // Serial.println((millis() - buttonStartTime) > 1000);
    if ((millis() - buttonStartTime) > 1000 && (digitalRead(19) == HIGH || digitalRead(17) == HIGH) && (millis() - prevChangeTime) > 50) {
      buttonDirection == 0 ? up() : down();
      prevChangeTime = millis();
    }

    if (!data_ready & !hold) continue;
    drawing = 1;
    sprite.fillSprite(TFT_BLACK);
    draw_grid();
    draw_data(pixel_data);
    sprite.pushSprite(0, 0);
    drawing = 0;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


void loop() {}

uint16_t auto_trigger(uint16_t *data, uint16_t n) {
  uint16_t max = 0;
  for (int i = 0; i < n / 2; i++) {
    if (data[i] > max) {
      max = data[i];
    }
  }
  return max & 0x0FFF;
}

void process_buf(uint16_t *buffer, uint16_t *dest) {
  // uint16_t *tmp = (uint16_t *)malloc(150 * sizeof(uint16_t));
  uint16_t s_div = time_divisions[s_div_idx];
  float sample_per_pixel = s_div / 30.0;

  uint8_t trigger = 0;
  for (int i = 1; i < 180; i++) {
    uint16_t cur = buf[(uint16_t)round(i * sample_per_pixel)] & 0x0FFF;
    uint16_t prev = buf[(uint16_t)round((i - 1) * sample_per_pixel)] & 0x0FFF;
    if ((cur > prev) && abs(cur - trigger_val) < 50) {
      trigger = i - 1;
      break;
    }
  }

  for (int i = 0; i < 120; i++) {
    uint16_t val_cur = buf[(uint16_t)round((i + trigger) * sample_per_pixel)];
    uint16_t scaled_val_cur = ((((val_cur & 0x0FFF) / 4095.0) * 3.3) / (voltage_divisions[v_div_idx] * 4)) * 120;
    dest[i] = scaled_val_cur;
  }
}

void draw_data(uint16_t *data) {
  for (int i = 1; i < 120; i++) {
    sprite.drawLine(i - 1, data[i - 1] - 123 < 0 ? abs(data[i - 1] - 123) : 123 - data[i - 1], i, data[i] - 123 < 0 ? abs(data[i] - 123) : 123 - data[i], TFT_YELLOW);
  }
}

void draw_grid() {
  for (int i = 0; i < 125; i += 5) {
    for (int j = 3; j <= 123; j += 30) {
      sprite.drawPixel(i, j, TFT_WHITE);
    }
  }
  for (int i = 3; i < 123; i += 5) {
    for (int j = 0; j < 125; j += 30) {
      sprite.drawPixel(j, i, TFT_WHITE);
    }
  }
  draw_info();
}

void draw_info() {
  sprite.setTextSize(1);

  sprite.fillRect(123, 2, 35, 20, TFT_DARKGREY);
  sprite.fillRect(123, 23, 35, 20,  TFT_DARKGREY);
  sprite.fillRect(123, 44, 35, 20,  TFT_DARKGREY);
  sprite.fillRect(123, 65, 35, 20,  TFT_DARKGREY);
  sprite.fillRect(123, 86, 35, 20,  TFT_DARKGREY);

  sprite.setTextColor(TFT_BLACK);

  sprite.drawString("s/Div", 125, 4);
  uint16_t s_div = time_divisions[s_div_idx];
  sprite.drawString(s_div >= 1000 ? (s_div == 2500 ? "2.5" : String(s_div / 1000)) : String(s_div), 125, 12);
  sprite.drawString(s_div >= 1000 ? "ms" : "us", 145, 12);

  float v_div = voltage_divisions[v_div_idx];
  sprite.drawString("v/Div", 125, 25);
  sprite.drawString(v_div == 0.5 ? "0.5" : String((byte)v_div), 125, 33);
  sprite.drawString("V", 145, 33);

  sprite.drawString("HOLD", 129, 51);
  if (hold) {
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString("HOLD", 3, 5);
    sprite.setTextColor(TFT_BLACK);
  }

  // sprite.drawString("TRIG", 129, 72);
  if (auto_trig) {
    sprite.drawString("AUTO", 129, 72);
  } else {
    sprite.drawString("MAN", 132, 72);
  }
  sprite.drawString(String((trigger_val / 4095.0) * 3.3), 129, 93);
  // sprite.drawString("INFO", 129, 93);

  sprite.setTextColor(TFT_WHITE);
  sprite.drawString("MAOTEK", 123, 109);
  sprite.drawString("scope", 125, 118);
  // sprite.setTextColor(TFT_WHITE);

  switch (menu) {
    case 0:
      sprite.drawRect(123, 2, 35, 20, TFT_WHITE);
      break;
    case 1:
      sprite.drawRect(123, 23, 35, 20, TFT_WHITE);
      break;
    case 2:
      sprite.drawRect(123, 44, 35, 20, TFT_WHITE);
      break;
    case 3:
      sprite.drawRect(123, 65, 35, 20, TFT_WHITE);
      break;
    case 4:
      sprite.drawRect(123, 86, 35, 20, TFT_WHITE);
      break;
  }
}