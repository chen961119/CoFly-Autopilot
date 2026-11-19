#include "math/units.h"
#include "config/aircraft_config.h"
#include "config/serial_config.h"
#include "interface/human_interface.h"
#include "system/flight_clock.h"
#include "system/rtos_sync.h"
#include <Adafruit_SSD1306.h>
#include "logging/logger.h"
#include <Wire.h>
#include "interaircraft_comm/interaircraft_comm.h"
#include "sensors/sensor_processing.h"
#include "sensors/sensor_calibration.h"
#include "system/flight_lock.h"
#include "actuators/actuator_output.h"
#include "parameters/parameter_service.h"

namespace {
// 按键相关
const int DEBOUNCE_DELAY = 10;              // 消抖时间(ms)
const unsigned long SHORT_PRESS_TIME = 50;  // 短按时间(ms)
const unsigned long LONG_PRESS_TIME = 1000; // 长按时间(ms)
struct Button {
  bool raw = false, down = false, longPress = false;
  uint32_t rawChanged = 0, pressedAt = 0;
};
Button modeButton, calibrationButton;
bool displayReady = false, slicedRefresh = false;
constexpr unsigned displayBytes = SCREEN_WIDTH * SCREEN_HEIGHT / 8;
unsigned displayOffset = displayBytes;

// OLED 显示器状态与初始化参数。
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


// 特殊图案
const unsigned char logo[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


unsigned long lastdispTime = 0;
unsigned int displayfreq = 10; // 显示屏帧率。

unsigned long blink_counter, blink_delay;
bool blinkAlternate;

void refreshDisplay() {
  if (!displayReady) return;
  if (slicedRefresh) displayOffset = 0;
  else display.display();
}
} // namespace

// 根据当前飞行模式设置 32、33 号模式指示灯电平。
void displayFlightModeIndicators(FlightMode mode) {
  if (mode == STABILIZE_MODE) {
    digitalWrite(33, HIGH);
    digitalWrite(32, HIGH);
  } else if (mode == MANUAL_MODE) {
    digitalWrite(33, HIGH);
    digitalWrite(32, LOW);
  }
}

// 初始化按键和 OLED。
void initializeHumanInterface() {
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(36, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  displayReady = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if (!displayReady) {
    USBSerial.println(F("SSD1306 allocation failed"));
    return;
  }
  display.clearDisplay();
  displaythumbsup();

}

// 按编译配置在 OLED 上显示本机 A～G 编号。
void displayAircraftIdentity() {
if ((aircraftId() == 1)) {
  displayID("A");
} else if ((aircraftId() == 2)) {
  displayID("B");
} else if ((aircraftId() == 3)) {
  displayID("C");
} else if ((aircraftId() == 4)) {
  displayID("D");
} else if ((aircraftId() == 5)) {
  displayID("E");
} else if ((aircraftId() == 6)) {
  displayID("F");
} else if ((aircraftId() == 7)) {
  displayID("G");
} else {
  displayID("?");
}
  if (!imuCalibrationValid()) displayAttitude();
}

// 按当前闪烁间隔更新板载 LED，供观察主循环是否仍在运行。
void loopBlink() {
  // 以长短交替的节奏闪烁板载 LED，表示主循环仍在运行。
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); // 13 号引脚连接板载 LED。

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

// 按指定次数闪烁板载 LED；upTime、downTime 单位为 ms。
void setupBlink(int numBlinks, int upTime, int downTime) {
  // 初始化阶段按指定次数和亮灭时间闪烁；时间单位为 ms。
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}


//=========================================================================================//

// 显示和打包辅助函数。

// 把传入的机体编号文字绘制到 OLED 指定位置并立即刷新。
void displayID(char *ss) {
  if (!displayReady) return;
  if (parameterRebootRequired()) { displayRebootRequired(); return; }
  // display.clearDisplay();
  display.setTextSize(2, 2);           // 使用双倍字体。
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.setCursor(100, 12);          // 指定文本起点。
  display.println(ss);                 // 直接用指针的形式。
  refreshDisplay();
}

// 把 SD 卡状态文字绘制到 OLED 指定位置并立即刷新。
void displaySD(char *ss) {
  if (!displayReady) return;
  if (parameterRebootRequired()) { displayRebootRequired(); return; }
  // display.clearDisplay();
  display.setTextSize(1);              // 使用默认字体大小。
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.setCursor(100, 0);           // 指定文本起点。
  display.println(ss);                 // 直接用指针的形式。
  // display.println(F("sdsds")); //用字符串
  refreshDisplay();
}

// 将当前日志编号写入 OLED 显示缓冲区。
void displayfilenum() {
  if (!displayReady) return;
  if (parameterRebootRequired()) { displayRebootRequired(); return; }
  if (!loggerSdReady()) {
    displaySD("MISS");
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.setCursor(100, 0);
  display.print(loggerFileNumber());
  display.print((char)247); // 度符号°
}

// 按显示频率刷新相对转角及姿态角；关闭显示时直接返回。
void displayRebootRequired() {
  if (!displayReady) return;
  static bool shown = false;
  if (shown) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(28, 0);
  display.print("REBOOT");
  display.setCursor(16, 16);
  display.print("REQUIRED");
  refreshDisplay();
  shown = true;
}

void displayAttitude() {
  if (!displayReady) return;
  if (parameterRebootRequired()) { displayRebootRequired(); return; }
  // Finish the current frame even when slow I2C transactions extend past 100 ms.
  if (slicedRefresh && displayOffset < displayBytes) return;

  if (!isFlightLocked()) {
    return;
  }
  float invFreq = 1.0 / displayfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastdispTime < invFreq)
    return;
  lastdispTime = checker;

  // 提醒占用左侧传感器区域，右侧日志编号与机体编号保持可见。
  display.fillRect(0, 0, 96, 32, SSD1306_BLACK);
  if (!aircraftConfigurationValid()) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("SET ID / COUNT");
    display.setCursor(0, 8);
    display.print("USB CONFIG");
    refreshDisplay();
    return;
  }
  if (!imuCalibrationValid()) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("IMU CAL");
    display.setCursor(0, 8);
    display.print("REQUIRED");
    display.setCursor(0, 16);
    display.print("Keep flat/still");
    refreshDisplay();
    return;
  }

  float attitude[3], relative;
  bool angleOk;
  {
    TaskCriticalSection guard;
    memcpy(attitude, att, sizeof(attitude));
    relative = relativeAngle_ready;
    angleOk = rotateSensorValid();
  }
  // 第一行：MT6701 减去零点后的相对角。
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 0, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 0);
  display.print("Relat: ");
  if (angleOk) {
    display.print(Units::toDegrees(relative), 1);
    display.print((char)247); // 度符号°
  } else {
    display.print("--");
  }

  // 第二行：滚转角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 8, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 8);
  display.print("Roll: ");
  display.print(Units::toDegrees(attitude[0]), 1);
  display.print((char)247); // 度符号°

  // 第三行：俯仰角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 16, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 16);
  display.print("Pitch:");
  display.print(Units::toDegrees(attitude[1]), 1);
  display.print((char)247);

  // 第四行：偏航角速度
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 24, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 24);
  display.print("Yaw:  ");
  display.print(Units::toDegrees(attitude[2]), 1);
  display.print((char)247);

  // 图形化指示（简易人工地平仪）
  // drawArtificialHorizon(roll, pitch);

  refreshDisplay();
}

// 显示启动位图，等待 1 秒后清空 OLED 缓冲区。
void displaythumbsup() {
  if (!displayReady) return;
  if (parameterRebootRequired()) { displayRebootRequired(); return; }
  display.drawBitmap(0,       // 居中X位置
                     0,       // 居中Y位置
                     logo,    // 位图数据
                     128, 32, // 宽度和高度
                     WHITE    // 颜色
  );

  // 显示绘制的内容
  refreshDisplay();

  // delay(1000);
  display.clearDisplay();
}

// The MT6701 and OLED are both on Wire1. Send at most 16 data bytes between
// sensor polls. A new frame starts with an explicit SSD1306 address window.
void serviceDisplayTransfer() {
  slicedRefresh = true;
  if (!displayReady || displayOffset >= displayBytes) return;
  Wire1.setClock(400000);
  if (displayOffset == 0) {
    const uint8_t window[] = {0, SSD1306_PAGEADDR, 0, SCREEN_HEIGHT / 8 - 1,
                              SSD1306_COLUMNADDR, 0, SCREEN_WIDTH - 1};
    Wire1.beginTransmission(SCREEN_ADDRESS);
    Wire1.write(window, sizeof(window));
    if (Wire1.endTransmission() != 0) displayOffset = displayBytes;
  }
  if (displayOffset < displayBytes) {
    constexpr unsigned chunk = 16;
    Wire1.beginTransmission(SCREEN_ADDRESS);
    Wire1.write(static_cast<uint8_t>(0x40));
    Wire1.write(display.getBuffer() + displayOffset, chunk);
    displayOffset = Wire1.endTransmission() == 0 ? displayOffset + chunk : displayBytes;
  }
  Wire1.setClock(100000);
}

namespace {
void pollButton(Button &button, unsigned pin, bool isModeButton) {
  const uint32_t now = millis();
  const bool raw = digitalRead(pin) == LOW;
  if (button.raw != raw) {
    button.raw = raw;
    button.rawChanged = now;
  }
  if (button.down != raw && now - button.rawChanged >= DEBOUNCE_DELAY) {
    button.down = raw;
    if (raw) {
      const char *message = isModeButton ? "按键按下\n" : "按键2按下\n";
      queueUsbDiagnosticOutput(message, strlen(message));
      button.pressedAt = button.rawChanged;
      button.longPress = false;
    } else {
      const char *message = isModeButton ? "按键释放\n" : "按键2释放\n";
      queueUsbDiagnosticOutput(message, strlen(message));
      if (isModeButton && !button.longPress && button.rawChanged - button.pressedAt >= SHORT_PRESS_TIME)
        startSensorCalibration("ROTATE");
    }
  }
  if (button.down && raw && !button.longPress && now - button.pressedAt >= LONG_PRESS_TIME) {
    button.longPress = true;
    if (isModeButton) {
      constexpr char message[] = "长按一次\n";
      queueUsbDiagnosticOutput(message, sizeof(message) - 1);
      setFlightLocked(!isFlightLocked());
    } else {
      requestButtonImuCalibration();
    }
  }
}
}

void ProcessButtonState() {
  pollButton(modeButton, 36, true);
  pollButton(calibrationButton, 31, false);
}
