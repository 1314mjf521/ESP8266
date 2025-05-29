/*
 * 项目名称: ESP8266 步进电机远程控制系统
 * Project Name: ESP8266 Stepper Motor Remote Control System
 *
 * 功能描述 (Function Description):
 * 1. 支持通过物理按钮控制步进电机的开关和方向。
 *    Supports physical buttons to control the motor's on/off state and direction.
 * 2. 支持通过网页接口控制电机的开关和方向。
 *    Provides a web interface to control the motor's on/off state and direction.
 * 3. 支持通过MQTT协议实现远程控制和状态上报。
 *    Enables remote control and status reporting via MQTT protocol.
 * 4. 支持智能配网功能，用户无需硬编码WiFi信息。
 *    Includes smart configuration for WiFi, eliminating the need for hardcoded credentials.
 * 5. 支持OTA(Over-The-Air)升级功能，方便远程更新固件。
 *    Supports OTA (Over-The-Air) updates for convenient firmware upgrades.
 *
 * 功能特点 (Features):
 * - 简单易用，适合初学者快速上手。
 *   Simple and user-friendly, suitable for beginners.
 * - 支持多种控制方式：物理按钮、网页接口、MQTT协议。
 *   Supports multiple control methods: physical buttons, web interface, and MQTT protocol.
 * - 高度可扩展，可根据需求添加更多功能。
 *   Highly extensible, allowing for additional features as needed.
 * - 提供详细的中英双语注释，便于理解和调试。
 *   Includes detailed bilingual comments for better understanding and debugging.
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> // 智能配网库 / Smart configuration library
#include <ArduinoOTA.h> // OTA升级库 / OTA update library
#include <EEPROM.h> // 引入EEPROM库，用于保存MQTT地址
#include <ESP8266HTTPClient.h> // 引入HTTPClient库，用于远程OTA升级
#include <map> // 引入map库，用于存储控制端信息 / Include map library for storing client info
#include <BasicStepperDriver.h> // 替换为正确的头文件

#define EEPROM_SIZE 512 // 定义EEPROM大小
#define MQTT_ADDRESS_OFFSET 0 // MQTT地址在EEPROM中的起始位置
#define MQTT_ADDRESS_MAX_LENGTH 100 // MQTT地址的最大长度
#define MICROSTEP_MODE_EEPROM_ADDR 200 // EEPROM保存细分模式的地址

char mqtt_server[MQTT_ADDRESS_MAX_LENGTH] = "192.168.1.100"; // 默认MQTT服务器地址

ESP8266WebServer server(80); // 确保 server 的声明在所有函数之前
WiFiClient espClient; // 确保 espClient 的声明在所有函数之前
PubSubClient client(espClient); // 确保 client 的声明在所有函数之前

// 函数声明 / Function declarations
void handleRoot(); // 处理主页请求 / Handle root request
void handleMotorOn(); // 处理开启电机请求 / Handle motor on request
void handleMotorOff(); // 处理关闭电机请求 / Handle motor off request
void handleMotorDirection(); // 处理切换电机方向请求 / Handle motor direction toggle request
void handleMotorAPI(); // 处理电机控制API请求 / Handle motor control API request
void handleVersionInfo(); // 处理获取版本信息请求 / Handle version info request
void reconnectMQTT(); // MQTT重连函数 / MQTT reconnect function
void handlePhysicalButtons(); // 处理物理按钮逻辑 / Handle physical button logic
void handleMQTTMotorControl(String message); // 处理MQTT消息：电机控制 / Handle MQTT message: motor control
void handleOTA(); // 处理OTA升级请求 / Handle OTA upgrade request
void handleOTAUpload(); // 处理OTA文件上传 / Handle OTA file upload
void handleOTARemote(); // 处理远程OTA升级 / Handle remote OTA upgrade
void handleToggleMQTTControl(); // 处理启用或禁用MQTT控制的请求 / Handle enable or disable MQTT control
void handleDeviceInfo(); // 处理设备信息请求 / Handle device info request
void handleSetMotorRunDuration(); // 处理设置电机启动时长的网页请求 / Handle web request to set motor run duration
void handleMotorButton(); // 处理电机按钮逻辑 / Handle motor button logic
void handleMotorRunDuration(); // 处理电机运行时长逻辑 / Handle motor run duration logic
void handleClientsPage(); // 处理控制端信息页面请求 / Handle client info page request
void handleSetClientName(); // 处理设置控制端名称的请求 / Handle set client name request
void updateClientOnlineStatus(String mac); // 更新控制端在线状态 / Update client online status
void handleResetWiFi(); // 处理重新配网请求 / Handle reset WiFi request
void handleSpeedUp(); // 处理加速请求 / Handle speed up request
void handleSlowDown(); // 处理减速请求 / Handle slow down request
void initializeMotorPwm(); // 声明初始化电机PWM信号的函数 / Declare the function to initialize motor PWM signal
void handleSetMicrostep(); // 处理设置细分模式的请求 / Handle set microstep mode request
void handleGetMicrostep(); // 处理获取当前细分模式的请求 / Handle get current microstep mode request
void setMicrostepMode(int microstep); // setMicrostepMode前向声明
void handleStepOnce(); // 新增单步运行接口，便于调试和外部调用
void handleApiStepOnce(); // 新增API接口：单步运行（API风格，支持GET/POST）

// WiFi连接失败计数器 / WiFi connection failure counter
int wifiConnectFailures = 0;

// 加载保存的MQTT地址 / Load the saved MQTT address from EEPROM
void loadMQTTAddress() {
  EEPROM.begin(EEPROM_SIZE); // 初始化EEPROM / Initialize EEPROM
  for (int i = 0; i < MQTT_ADDRESS_MAX_LENGTH; i++) {
    mqtt_server[i] = EEPROM.read(MQTT_ADDRESS_OFFSET + i); // 从EEPROM读取字符 / Read characters from EEPROM
    if (mqtt_server[i] == '\0') break; // 遇到字符串结束符停止读取 / Stop reading at null terminator
  }
  if (mqtt_server[0] == '\0') {
    strcpy(mqtt_server, "192.168.1.100"); // 如果未设置地址，使用默认值 / Use default if no address is set
  }
  Serial.println("加载的MQTT地址: " + String(mqtt_server)); // 打印加载的地址 / Print the loaded address
}

// 保存MQTT地址到EEPROM / Save the MQTT address to EEPROM
void saveMQTTAddress(const char* address) {
  for (int i = 0; i < MQTT_ADDRESS_MAX_LENGTH; i++) {
    EEPROM.write(MQTT_ADDRESS_OFFSET + i, address[i]); // 写入字符到EEPROM / Write characters to EEPROM
    if (address[i] == '\0') break; // 遇到字符串结束符停止写入 / Stop writing at null terminator
  }
  EEPROM.commit(); // 提交更改 / Commit changes
  Serial.println("保存的MQTT地址: " + String(address)); // 打印保存的地址 / Print the saved address
}

// 处理设置MQTT地址的网页请求 / Handle web request to set MQTT address
void handleSetMQTTAddress() {
  if (server.hasArg("address")) { // 检查是否提供了地址参数 / Check if address parameter is provided
    String address = server.arg("address");
    if (address.length() < MQTT_ADDRESS_MAX_LENGTH) {
      address.toCharArray(mqtt_server, MQTT_ADDRESS_MAX_LENGTH); // 将地址转换为字符数组 / Convert address to char array
      saveMQTTAddress(mqtt_server); // 保存地址到EEPROM / Save address to EEPROM
      client.setServer(mqtt_server, 1883); // 更新MQTT服务器地址 / Update MQTT server address
      Serial.printf("MQTT地址已更新为: %s / MQTT address updated to: %s\n", mqtt_server, mqtt_server);
      server.send(200, "text/plain; charset=utf-8", "MQTT地址已更新 / MQTT address updated");
    } else {
      server.send(400, "text/plain; charset=utf-8", "地址过长 / Address too long");
      Serial.println("设置MQTT地址失败：地址过长 / Failed to set MQTT address: Address too long");
    }
  } else {
    server.send(400, "text/plain; charset=utf-8", "缺少地址参数 / Missing address parameter");
    Serial.println("设置MQTT地址失败：缺少地址参数 / Failed to set MQTT address: Missing address parameter");
  }
}

// 检查是否定义了引脚宏，如果未定义则手动定义
#ifndef D1
#define D1 4 // 替换为实际的 GPIO 编号
#endif
#ifndef D2
#define D2 5 // 替换为实际的 GPIO 编号
#endif
#ifndef D3
#define D3 16 // 替换为实际的 GPIO 编号
#endif
#ifndef D4
#define D4 14 // 替换为实际的 GPIO 编号
#endif
#ifndef D5
#define D5 12 // 替换为实际的 GPIO 编号
#endif
#ifndef D6
#define D6 13 // 替换为实际的 GPIO 编号闲置引脚暂未使用
#endif

// 引脚定义区域 / Pin definitions
// =============================
// 步进电机控制引脚定义 / Stepper motor control pin definitions
#define DIR_PIN D1  // 方向控制引脚 / Direction control pin
#define STEP_PIN D2 // 步进控制引脚 / Step control pin
#define ENABLE_PIN D3 // 使能引脚 / Enable pin

// 电机开关及限位按钮引脚定义 / Motor toggle and limit switch pin definition
#define MOTOR_BUTTON_PIN D4 // 电机开关及限位按钮引脚 / Motor toggle and limit switch pin

// 物理按钮引脚定义 / Physical button pin definitions
#define BUTTON_DIRECTION_PIN D5 // 电机方向按钮引脚 / Motor direction button pin

// 板载 LED 引脚定义 / Onboard LED pin definition
#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // 通常 ESP8266 的板载 LED 位于 GPIO 2 / Typically onboard LED is on GPIO 2
#endif
// =============================

// 按钮状态变量 / Button state variables
bool lastMotorButtonState = HIGH; // 上一次读取的电机按钮状态 / Last read state of the motor button
unsigned long lastMotorButtonDebounceTime = 0; // 防抖时间戳 / Debounce timestamp
const unsigned long motorButtonDebounceDelay = 50; // 防抖延迟（毫秒） / Debounce delay (milliseconds)

bool lastEnableButtonState = HIGH; // 上一次读取的电机开关按钮状态 / Last read state of the motor on/off button
bool lastDirectionButtonState = HIGH; // 上一次读取的电机方向按钮状态 / Last read state of the motor direction button
unsigned long lastDebounceTime = 0; // 防抖时间戳 / Debounce timestamp
const unsigned long debounceDelay = 50; // 防抖延迟（毫秒） / Debounce delay (milliseconds)

// MQTT主题定义 / MQTT topic definitions
const char* mqtt_topic_motor_control = "motor/control"; // 电机控制主题 / Motor control topic
const char* mqtt_topic_status_report = "motor/status";  // 状态上报主题 / Status report topic

// 电机状态变量 / Motor state variables
bool motorEnabled = false; // 电机是否开启 / Whether the motor is enabled
bool motorDirection = true; // 电机方向：true为正转，false为反转 / Motor direction: true for forward, false for reverse

// 电机启动时长（毫秒） / Motor run duration (milliseconds)
unsigned long motorRunDuration = 10000; // 默认10秒 / Default 10 seconds
unsigned long motorStartTime = 0; // 电机启动时间戳 / Motor start timestamp

// 电机未使用超时时间（毫秒） / Motor inactivity timeout (milliseconds)
const unsigned long motorInactivityTimeout = 5 * 60 * 1000; // 5 分钟 / 5 minutes
unsigned long lastMotorActivityTime = 0; // 上次电机操作时间戳 / Last motor activity timestamp

// 定义版本号 / Define version number
#define FIRMWARE_VERSION "1.0.2"

// 控制端上线标志 / Controller online flag
bool controllerOnline = false;

// 控制端检测的时间间隔（毫秒） / Controller detection interval (milliseconds)
const unsigned long controllerCheckInterval = 5000; // 每5秒检测一次 / Check every 5 seconds
unsigned long lastControllerCheckTime = 0; // 上次检测时间戳 / Last check timestamp

// 控制端信息存储 / Client information storage
std::map<String, String> clients; // 存储控制端MAC地址和名称 / Store client MAC address and name

// 动态记录控制端MAC地址 / Dynamically record controller MAC addresses
void handleRegisterController() {
  String macAddress = WiFi.macAddress();
  Serial.println("控制端已注册，MAC地址: / Controller registered, MAC address: " + macAddress);
  controllerOnline = true; // 设置控制端上线标志 / Set controller online flag
  server.send(200, "text/plain", "控制端已注册 / Controller registered");
}

// MQTT消息回调函数声明 / MQTT message callback function declaration
void mqttCallback(char* topic, byte* payload, unsigned int length);

// 定义MQTT重连的时间间隔（毫秒） / Define MQTT reconnect interval (milliseconds)
const unsigned long mqttReconnectInterval = 5000; // 每5秒尝试一次 / Retry every 5 seconds
unsigned long lastMQTTReconnectAttempt = 0; // 上次尝试时间戳 / Last reconnect attempt timestamp

// MQTT控制启用标志 / MQTT control enable flag
bool mqttControlEnabled = false; // 默认禁用MQTT控制 / Default to disabled

// MQTT重连函数 / MQTT reconnect function
void reconnectMQTT() {
  if (!mqttControlEnabled) {
    Serial.println("MQTT控制已禁用，跳过重连 / MQTT control disabled, skipping reconnect");
    return;
  }

  if (client.connected()) return; // 如果已连接，直接返回 / Return if already connected

  unsigned long now = millis();
  if (!controllerOnline) {
    // 限制串口输出频率 / Limit serial output frequency
    if (now - lastControllerCheckTime >= controllerCheckInterval) {
      lastControllerCheckTime = now; // 更新上次检测时间戳 / Update last check timestamp
      Serial.println("控制端未上线，跳过MQTT连接 / Controller not online, skipping MQTT connection");
    }
    return;
  }

  if (now - lastMQTTReconnectAttempt >= mqttReconnectInterval) {
    lastMQTTReconnectAttempt = now; // 更新上次尝试时间戳 / Update last attempt timestamp
    Serial.print("尝试连接MQTT服务器... / Attempting to connect to MQTT server...");
    if (client.connect("ESP8266Client")) {
      Serial.println("连接成功 / Connected");
      client.subscribe(mqtt_topic_motor_control); // 订阅电机控制主题 / Subscribe to motor control topic
      client.subscribe("motor/step_once"); // 新增订阅单步运行主题
    } else {
      Serial.print("连接失败，状态码= / Connection failed, state=");
      Serial.println(client.state());
    }
  }
}

// 嵌入的HTML内容 / Embedded HTML content
const char MAIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP8266 步进电机控制系统</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 20px; }
    h1 { color: #333; }
    button, input[type="text"], input[type="number"] { padding: 10px 20px; margin: 10px; font-size: 16px; }
    button { background-color: #4CAF50; color: white; border: none; cursor: pointer; }
    button:hover { background-color: #45a049; }
    input[type="text"], input[type="number"] { width: 300px; }
    .info { margin: 20px; font-size: 18px; }
    .button-group { margin: 20px; }
  </style>
  <script>
    // 加载设备信息 / Load device information
    function loadDeviceInfo() {
      fetch('/api/device_info')
        .then(response => response.json())
        .then(data => {
          document.getElementById('ipAddress').innerText = data.ip;
          document.getElementById('macAddress').innerText = data.mac;
          document.getElementById('version').innerText = data.version;
          document.getElementById('onlineClients').innerText = data.onlineClients;
        })
        .catch(() => alert('无法加载设备信息 / Failed to load device information'));
    }

    // 设置电机启动时长 / Set motor run duration
    function setMotorDuration() {
      const duration = document.getElementById('motorDuration').value;
      fetch(`/api/set_motor_duration?duration=${duration}`)
        .then(response => {
          if (response.ok) {
            alert('电机启动时长已更新！ / Motor run duration updated!');
          } else {
            alert('设置失败，请检查输入值 / Failed to set duration, please check the input value');
          }
        })
        .catch(() => alert('无法连接到设备 / Unable to connect to the device'));
    }

    // 设置MQTT地址 / Set MQTT address
    function setMQTTAddress() {
      const address = document.getElementById('mqttAddress').value;
      fetch(`/api/set_mqtt?address=${encodeURIComponent(address)}`)
        .then(response => {
          if (response.ok) {
            alert('MQTT地址已更新！ / MQTT address updated!');
          } else {
            alert('设置失败，请检查输入值 / Failed to set MQTT address, please check the input value');
          }
        })
        .catch(() => alert('无法连接到设备 / Unable to connect to the device'));
    }

    // 切换MQTT控制 / Toggle MQTT control
    function toggleMQTTControl(enable) {
      const url = `/api/mqtt_control?enable=${enable}`;
      fetch(url)
        .then(response => {
          if (response.ok) {
            alert(enable ? 'MQTT控制已启用！' : 'MQTT控制已禁用！');
          } else {
            alert('操作失败，请检查设备状态 / Operation failed, please check device status');
          }
        })
        .catch(() => alert('无法连接到设备 / Unable to connect to the device'));
    }

    // 重新进入配网模式 / Reset WiFi configuration
    function resetWiFi() {
      fetch('/api/reset_wifi')
        .then(response => {
          if (response.ok) {
            alert('设备正在重新进入配网模式 / Device is restarting to enter configuration mode');
          } else {
            alert('操作失败，请检查设备状态 / Operation failed, please check device status');
          }
        })
        .catch(() => alert('无法连接到设备 / Unable to connect to the device'));
    }

    // 加速电机 / Speed up motor
    function speedUpMotor() {
      fetch('/motor/speed_up')
        .then(response => {
          if (response.ok) {
            alert('电机加速成功！ / Motor speed increased successfully!');
          } else {
            alert('加速失败！ / Speed up failed!');
          }
        })
        .catch(() => alert('无法连接到设备 / Unable to connect to the device'));
    }

    // 减速电机 / Slow down motor
    function slowDownMotor() {
      fetch('/motor/slow_down')
        .then(response => {
          if (response.ok) {
            alert('电机减速成功！ / Motor speed decreased successfully!');
          } else {
            alert('减速失败！ / Slow down failed!');
          }
        })
        .catch(() => alert('无法连接到设备 / Unable to connect to the device'));
    }

    // 单步运行
    function stepOnce() {
      fetch('/motor/step_once')
        .then(response => {
          if (response.ok) {
            alert('单步运行已执行！ / Step motor once executed!');
          } else {
            alert('单步运行失败！ / Step motor once failed!');
          }
        })
        .catch(() => alert('无法连接到设备 / Unable to connect to the device'));
    }

    window.onload = loadDeviceInfo;
  </script>
</head>
<body>
  %MICROSTEP_OPTIONS%
  <h1>ESP8266 步进电机控制系统</h1>
  <div class="info">
    <p>设备IP地址: <strong id="ipAddress"></strong></p>
    <p>设备MAC地址: <strong id="macAddress"></strong></p>
    <p>固件版本: <strong id="version"></strong></p>
    <p>在线控制端数量: <strong id="onlineClients"></strong></p>
    <button onclick="location.href='/clients'">查看控制端信息</button>
  </div>
  <div class="button-group">
    <h2>电机控制</h2>
    <button onclick="fetch('/motor/on').then(() => alert('电机已开启！')).catch(() => alert('操作失败！'));">开启电机</button>
    <button onclick="fetch('/motor/off').then(() => alert('电机已关闭！')).catch(() => alert('操作失败！'));">关闭电机</button>
    <button onclick="fetch('/motor/direction').then(() => alert('电机方向已切换！')).catch(() => alert('操作失败！'));">切换电机方向</button>
    <button onclick="speedUpMotor()">加速</button>
    <button onclick="slowDownMotor()">减速</button>
    <button onclick="stepOnce()">单步运行</button>
  </div>
  <div class="button-group">
    <h2>设置电机启动时长</h2>
    <input type="number" id="motorDuration" placeholder="输入时长（秒）" min="1" max="1800" required>
    <button onclick="setMotorDuration()">设置时长</button>
  </div>
  <div class="button-group">
    <h2>设置 MQTT 地址</h2>
    <input type="text" id="mqttAddress" placeholder="输入MQTT服务器地址" required>
    <button onclick="setMQTTAddress()">更新地址</button>
  </div>
  <div class="button-group">
    <h2>MQTT 控制</h2>
    <button onclick="toggleMQTTControl(true)">启用 MQTT 控制</button>
    <button onclick="toggleMQTTControl(false)">禁用 MQTT 控制</button>
  </div>
  <div class="button-group">
    <h2>OTA 升级</h2>
    <button onclick="location.href='/ota'">开始OTA升级</button>
  </div>
  <div class="button-group">
    <h2>WiFi 配置</h2>
    <button onclick="resetWiFi()">重新进入配网模式</button>
  </div>
</body>
</html>
)rawliteral";

// 获取当前时间戳（毫秒） / Get current timestamp (milliseconds)
unsigned long getTimestamp() {
  return millis();
}

// 更新电机活动时间戳 / Update motor activity timestamp
void updateMotorActivity() {
  lastMotorActivityTime = millis();
  Serial.printf("[%lu] 电机活动时间已更新 / Motor activity timestamp updated\n", lastMotorActivityTime);
}

// 检查电机未使用超时 / Check motor inactivity timeout
void checkMotorInactivity() {
  if (motorEnabled && (millis() - lastMotorActivityTime >= motorInactivityTimeout)) {
    motorEnabled = false;
    digitalWrite(ENABLE_PIN, HIGH); // 禁用电机 / Disable motor
    Serial.printf("[%lu] 电机因未使用超时已禁用 / Motor disabled due to inactivity timeout\n", millis());
  }
}

// LED 状态变量 / LED state variables
bool ledState = LOW;
unsigned long lastLedToggleTime = 0;
const unsigned long ledBlinkInterval = 500; // LED 频闪间隔（毫秒） / LED blink interval (milliseconds)

// 更新 LED 状态的函数 / Function to update LED state
void updateLEDState() {
    unsigned long currentMillis = millis();

    // 如果 WiFi 已连接，LED 常亮 / If WiFi is connected, LED stays on
    if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(LED_BUILTIN, LOW); // 常亮（低电平点亮） / LED on (active LOW)
        return;
    }

    // 如果 WiFi 未连接，LED 频闪 / If WiFi is not connected, LED blinks
    if (currentMillis - lastLedToggleTime >= ledBlinkInterval) {
        lastLedToggleTime = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH); // 切换 LED 状态 / Toggle LED state
    }
}

// 初始化WiFi连接 / Initialize WiFi connection with fault tolerance
void initializeWiFi() {
    pinMode(LED_BUILTIN, OUTPUT); // 设置板载 LED 为输出模式 / Set onboard LED as output
    digitalWrite(LED_BUILTIN, HIGH); // 默认关闭 LED（高电平熄灭） / Default LED off (active LOW)

    WiFiManager wifiManager;
    wifiManager.setTimeout(180); // 设置智能配网超时时间为180秒 / Set smart configuration timeout to 180 seconds

    while (wifiConnectFailures < 3) {
        if (wifiManager.autoConnect("ESP8266_SmartConfig")) { // 设置配网热点名称 / Set WiFi configuration hotspot name
            Serial.println("WiFi连接成功 / WiFi connected");
            Serial.print("设备IP地址: / Device IP Address: ");
            Serial.println(WiFi.localIP());
            return;
        } else {
            wifiConnectFailures++;
            Serial.printf("WiFi连接失败，第%d次尝试 / WiFi connection failed, attempt %d\n", wifiConnectFailures, wifiConnectFailures);
            delay(10000); // 每次失败后等待10秒 / Wait 10 seconds after each failure
        }
    }

    // 超过3次失败后重新进入智能配网模式 / Enter smart configuration mode after 3 failures
    Serial.println("WiFi连接失败超过3次，进入智能配网模式 / WiFi connection failed more than 3 times, entering smart configuration mode");
    wifiManager.startConfigPortal("ESP8266_SmartConfig");
}

// 初始化Web服务器 / Initialize web server
void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/motor/on", handleMotorOn); // 处理开启电机请求 / Handle motor on request
  server.on("/motor/off", handleMotorOff); //  处理关闭电机请求 / Handle motor off request
  server.on("/motor/direction", handleMotorDirection); // 处理切换电机方向请求 / Handle motor direction toggle request
  server.on("/motor/speed_up", handleSpeedUp); // 添加加速接口 / Add speed up endpoint
  server.on("/motor/slow_down", handleSlowDown); // 添加减速接口 / Add slow down endpoint
  server.on("/api/motor", handleMotorAPI); // 添加API接口处理 / Add API endpoint handling
  server.on("/api/register", handleRegisterController); // 添加注册控制端的API接口 / Add API endpoint for registering controllers
  server.on("/api/version", handleVersionInfo); // 添加获取版本信息的API接口 / Add API endpoint for getting version information
  server.on("/ota", handleOTA); // 添加OTA升级接口 / Add OTA upgrade endpoint
  server.on("/ota/upload", HTTP_POST, []() {}, handleOTAUpload); // OTA文件上传 / OTA file upload
  server.on("/ota/remote", HTTP_POST, handleOTARemote); // 远程OTA升级 / Remote OTA upgrade
  server.on("/api/mqtt_control", handleToggleMQTTControl); // 添加MQTT控制开关接口 / Add MQTT control toggle endpoint
  server.on("/api/device_info", handleDeviceInfo); // 设备信息接口 / Device info API
  server.on("/api/set_motor_duration", handleSetMotorRunDuration); // 添加设置电机启动时长接口 / Add motor run duration API
  server.on("/api/set_mqtt", handleSetMQTTAddress); // 修复未注册的接口 / Fix unregistered endpoint
  server.on("/clients", handleClientsPage); // 控制端信息页面 / Client info page
  server.on("/api/set_client_name", handleSetClientName); // 设置控制端名称接口 / Set client name API
  server.on("/api/reset_wifi", handleResetWiFi); // 新增重新配网接口 / Add reset WiFi API
  server.on("/api/set_microstep", handleSetMicrostep); // 添加设置细分模式接口 / Add set microstep mode API
  server.on("/api/get_microstep", handleGetMicrostep); // 添加获取当前细分模式接口 / Add get current microstep mode API
  server.on("/motor/step_once", handleStepOnce); // 新增单步运行接口
  server.on("/api/step_once", HTTP_ANY, handleApiStepOnce); // RESTful API接口
  server.begin();
  Serial.println("Web服务器已启动 / Web server started");
}

// 步进电机参数
const int STEPS_PER_REV = 200; // 42步进电机一圈200步（1.8度/步）
const int MOTOR_RPM = 60;      // 默认转速，可根据需要调整
int pulsesPerRev = STEPS_PER_REV * 16; // 当前每圈脉冲数

// 步进电机速度参数
unsigned int stepInterval = 200; // 默认脉冲间隔（us），对应5kHz，适合细分
unsigned int stepIntervalMin = 50;   // 最快（20kHz）
unsigned int stepIntervalMax = 2000; // 最慢（500Hz）
bool stepDir = true; // true: HIGH, false: LOW

// 步进电机方向
// motorDirection 变量已存在，true=HIGH正转，false=LOW反转

// 步进电机运行状态
// motorEnabled 变量已存在

// BasicStepperDriver对象
BasicStepperDriver stepper(STEPS_PER_REV, DIR_PIN, STEP_PIN, ENABLE_PIN);

// 细分模式相关定义
enum MicrostepMode {
    MICROSTEP_FULL = 1,
    MICROSTEP_8 = 8,
    MICROSTEP_16 = 16,
    MICROSTEP_32 = 32
};

// 当前细分模式，默认16细分
int currentMicrostep = MICROSTEP_16;

// 保存/加载细分模式
void saveMicrostepMode(int microstep) {
    EEPROM.write(MICROSTEP_MODE_EEPROM_ADDR, microstep);
    EEPROM.commit();
}
int loadMicrostepMode() {
    int val = EEPROM.read(MICROSTEP_MODE_EEPROM_ADDR);
    if (val == MICROSTEP_FULL || val == MICROSTEP_8 || val == MICROSTEP_16 || val == MICROSTEP_32) {
        return val;
    }
    return MICROSTEP_16; // 默认16细分
}

// 优化：根据当前细分模式动态调整允许的最小脉冲间隔
void updateStepIntervalRange() {
    // 这些变量必须是可写的（去掉const），否则不能赋值
    if (currentMicrostep == MICROSTEP_FULL) {
        stepIntervalMin = 800; // 全步进时建议最小脉冲间隔800us（约1250Hz），防止丢步
    } else if (currentMicrostep == MICROSTEP_8) {
        stepIntervalMin = 200; // 8细分时建议最小200us
    } else if (currentMicrostep == MICROSTEP_16) {
        stepIntervalMin = 100; // 16细分时建议最小100us
    } else if (currentMicrostep == MICROSTEP_32) {
        stepIntervalMin = 50;  // 32细分时可更快
    }
    // stepIntervalMax 可保持不变
    if (stepInterval < stepIntervalMin) stepInterval = stepIntervalMin;
    if (stepInterval > stepIntervalMax) stepInterval = stepIntervalMax;
}

// 设置细分模式并重配置驱动
void setMicrostepMode(int microstep) {
    currentMicrostep = microstep;
    pulsesPerRev = STEPS_PER_REV * microstep;
    stepper.begin(MOTOR_RPM, microstep);
    stepper.setEnableActiveState(LOW);
    stepper.enable();
    saveMicrostepMode(microstep); // 每次切换细分都保存
    updateStepIntervalRange(); // 动态调整脉冲间隔范围
    Serial.printf("已切换细分模式: %d, 每圈脉冲数: %d, 最小脉冲间隔: %u us\n", microstep, pulsesPerRev, stepIntervalMin);
}

// 步进电机单步函数（兼容全步进和细分，脉冲宽度建议>2us，A4988推荐10-20us，过短可能全步进失效）
void stepMotorOnce() {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(20); // 兼容全步进和细分，20us更安全
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(20);
}

// 步进电机持续运行函数（使用BasicStepperDriver库）
// 若全步进模式下速度过快，建议用户适当调慢stepInterval
void runStepper() {
    static unsigned long lastStepTime = 0;
    if (!motorEnabled) {
        stepper.disable();
        return;
    }
    stepper.enable();
    unsigned long now = micros();
    if (now - lastStepTime >= stepInterval) {
        lastStepTime = now;
        stepper.move(stepDir ? 1 : -1);
    }
}

// 加速/减速接口（自动限制在当前细分允许的范围内）
void adjustMotorSpeed(bool increase) {
    if (increase) {
        if (stepInterval > stepIntervalMin) stepInterval -= 10;
        if (stepInterval < stepIntervalMin) stepInterval = stepIntervalMin;
    } else {
        if (stepInterval < stepIntervalMax) stepInterval += 10;
        if (stepInterval > stepIntervalMax) stepInterval = stepIntervalMax;
    }
    float rps = 1000000.0f / (stepInterval * pulsesPerRev);
    Serial.printf("当前脉冲间隔: %u us, 约 %.2f 转/秒\n", stepInterval, rps);
}

// 网页端细分模式选择表单
const char* MICROSTEP_OPTIONS_HTML = R"rawliteral(
  <form id="microstepForm" style="margin:20px;">
    <label>步进模式选择：</label>
    <select id="microstepSelect">
      <option value="1">全步进</option>
      <option value="8">8细分</option>
      <option value="16">16细分</option>
      <option value="32">32细分</option>
    </select>
    <button type="button" onclick="setMicrostep()">切换模式</button>
  </form>
  <script>
    function setMicrostep() {
      var val = document.getElementById('microstepSelect').value;
      fetch('/api/set_microstep?mode=' + val)
        .then(response => {
          if(response.ok) alert('细分模式已切换');
          else alert('切换失败');
        });
    }
    // 页面加载时设置当前选中
    window.addEventListener('DOMContentLoaded', function() {
      fetch('/api/get_microstep')
        .then(r=>r.json()).then(d=>{
          document.getElementById('microstepSelect').value = d.mode;
        });
    });
  </script>
)rawliteral";

// 初始化函数 / Initialization function
void setup() {
    EEPROM.begin(EEPROM_SIZE);
    Serial.begin(115200);
    Serial.println("ESP8266 步进电机控制系统启动");
    Serial.printf("请确保A4988的MS1/MS2/MS3全部为高电平，已设置为16细分，脉冲/圈=%d\n", pulsesPerRev);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH); // 默认关闭电机
    pinMode(MOTOR_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUTTON_DIRECTION_PIN, INPUT_PULLUP);

    // 加载细分模式并初始化驱动（确保调用）
    currentMicrostep = loadMicrostepMode();
    setMicrostepMode(currentMicrostep);

    // ...existing code...
    stepper.begin(MOTOR_RPM, currentMicrostep);
    stepper.setEnableActiveState(LOW); // A4988 EN低电平有效
    stepper.enable();
    currentMicrostep = loadMicrostepMode();
    setMicrostepMode(currentMicrostep);
   // 打印版本信息 / Print version information
   Serial.println("ESP8266 步进电机远程控制系统 / ESP8266 Stepper Motor Remote Control System");
   Serial.println("固件版本: " FIRMWARE_VERSION " / Firmware Version: " FIRMWARE_VERSION);

   pinMode(DIR_PIN, OUTPUT);
   pinMode(STEP_PIN, OUTPUT);
   pinMode(ENABLE_PIN, OUTPUT);
   digitalWrite(ENABLE_PIN, HIGH); // 默认关闭电机 / Default to motor off

   // 初始化物理按钮引脚 / Initialize physical button pins
   pinMode(MOTOR_BUTTON_PIN, INPUT_PULLUP); // 初始化电机按钮引脚 / Initialize motor button pin
   Serial.println("电机按钮初始化完成 / Motor button initialized");

   pinMode(BUTTON_DIRECTION_PIN, INPUT_PULLUP); // 使用内部上拉电阻 / Use internal pull-up resistor
   Serial.println("物理按钮初始化完成 / Physical buttons initialized");

   // 初始化WiFi连接 / Initialize WiFi connection
   initializeWiFi();

   // 加载保存的MQTT地址
   loadMQTTAddress(); // 加载保存的MQTT地址
   client.setServer(mqtt_server, 1883); // 设置MQTT服务器地址

   // 初始化MQTT / Initialize MQTT
   client.setCallback(mqttCallback); // 设置MQTT回调函数 / Set MQTT callback function

   // 初始化Web服务器 / Initialize web server
   setupWebServer();

   // 初始化OTA / Initialize OTA
   ArduinoOTA.onStart([]() {
     String type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem";
     Serial.println("开始OTA更新: " + type + " / Starting OTA update: " + type);
   });
   ArduinoOTA.onEnd([]() {
     Serial.println("\nOTA更新完成 / OTA update completed");
   });
   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     // 修复格式化字符串问题 / Fix format string issue
     Serial.printf("OTA更新进度: %u%%\r / OTA update progress: %u%%\r", progress * 100 / total, progress * 100 / total);
   });
   ArduinoOTA.onError([](ota_error_t error) {
     // 修复格式化字符串问题 / Fix format string issue
     Serial.printf("OTA更新错误[%u]: / OTA update error [%u]: ", static_cast<unsigned int>(error), static_cast<unsigned int>(error));
     if (error == OTA_AUTH_ERROR) {
       Serial.println("认证失败 / Authentication failed");
     } else if (error == OTA_BEGIN_ERROR) {
       Serial.println("开始失败 / Begin failed");
     } else if (error == OTA_CONNECT_ERROR) {
       Serial.println("连接失败 / Connection failed");
     } else if (error == OTA_RECEIVE_ERROR) {
       Serial.println("接收失败 / Receive failed");
     } else if (error == OTA_END_ERROR) {
       Serial.println("结束失败 / End failed");
     }
   }); // 修复结束符号 / Fix closing brace
   ArduinoOTA.begin();
   Serial.println("OTA功能已启动 / OTA functionality started");
}

// 主循环 / Main loop
void loop() {
  updateLEDState(); // 更新 LED 状态 / Update LED state
  handleMotorButton(); // 处理电机按钮逻辑 / Handle motor button logic
  handleMotorRunDuration(); // 处理电机运行时长逻辑 / Handle motor run duration logic
  checkMotorInactivity(); // 检查电机未使用超时 / Check motor inactivity timeout

  // 更新电机步进脉冲信号 / Update motor step pulse signal
  runStepper();

  // 检查物理按钮状态 / Check physical button states
  handlePhysicalButtons();

  // 检查MQTT连接状态，仅在启用时检测 / Check MQTT connection status only when enabled
  if (mqttControlEnabled) {
    if (!client.connected()) {
      reconnectMQTT(); // 尝试重新连接MQTT / Attempt to reconnect to MQTT
    }
    client.loop(); // 处理MQTT消息 / Handle MQTT messages
  } else {
    // 如果禁用MQTT控制，确保不会尝试连接或处理消息 / Ensure no connection or message handling when disabled
    static bool mqttDisabledLogged = false;
    if (!mqttDisabledLogged) {
      Serial.println("MQTT控制已禁用，跳过MQTT检测 / MQTT control disabled, skipping MQTT checks");
      mqttDisabledLogged = true;
    }
  }

  server.handleClient(); // 处理网页请求 / Handle web requests
  ArduinoOTA.handle(); // 处理OTA更新 / Handle OTA updates
}

// 处理加速请求 / Handle speed up request
void handleSpeedUp() {
  adjustMotorSpeed(true); // 加速 / Increase speed
  server.send(200, "text/plain; charset=utf-8", "电机加速 / Motor speed increased");
}

// 处理减速请求 / Handle slow down request
void handleSlowDown() {
  adjustMotorSpeed(false); // 减速 / Decrease speed
  server.send(200, "text/plain; charset=utf-8", "电机减速 / Motor speed decreased");
}

// 合并重复的 mqttCallback 函数定义 / Merge duplicate mqttCallback definitions
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (!mqttControlEnabled) {
        Serial.printf("[%lu] MQTT控制已禁用，忽略消息 / MQTT control disabled, ignoring message\n", millis());
        return;
    }
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.printf("收到 MQTT 消息，主题: %s，内容: %s\n", topic, message.c_str());

    // 处理电机控制主题的消息 / Handle motor control topic messages
    if (String(topic) == mqtt_topic_motor_control) {
        if (message == "forward") {
            digitalWrite(DIR_PIN, HIGH); // 设置为正向运动 / Set to forward
            digitalWrite(ENABLE_PIN, LOW); // 启动电机 / Enable motor
            Serial.println("电机正向运动（通过 MQTT） / Motor moving forward (via MQTT)");
        } else if (message == "reverse") {
            digitalWrite(DIR_PIN, LOW); // 设置为反向运动 / Set to reverse
            digitalWrite(ENABLE_PIN, LOW); // 启动电机 / Enable motor
            Serial.println("电机反向运动（通过 MQTT） / Motor moving reverse (via MQTT)");
        } else if (message == "off") {
            digitalWrite(ENABLE_PIN, HIGH); // 停止电机 / Disable motor
            Serial.println("电机停止（通过 MQTT） / Motor stopped (via MQTT)");
        }
    } else if (String(topic) == "motor/step_once") {
        stepMotorOnce();
        Serial.println("收到MQTT单步运行指令 / Step motor once by MQTT");
    } else {
        Serial.printf("未处理的 MQTT 主题: %s\n", topic);
    }
}

// 处理物理按钮逻辑 / Handle physical button logic
void handlePhysicalButtons() {
  // 读取电机方向按钮状态 / Read motor direction button state
  bool currentDirectionButtonState = digitalRead(BUTTON_DIRECTION_PIN);
  if (currentDirectionButtonState != lastDirectionButtonState) {
    if (currentDirectionButtonState == LOW) { // 按钮被按下 / Button pressed
      motorDirection = !motorDirection; // 切换电机方向 / Toggle motor direction
      stepDir = motorDirection;
      digitalWrite(DIR_PIN, stepDir ? HIGH : LOW); // 控制电机方向引脚 / Control motor direction pin
      Serial.printf("[%lu] 电机方向已切换为: %s（通过按钮） / Motor direction toggled to: %s (via button)\n", millis(), motorDirection ? "正转 / Forward" : "反转 / Reverse", motorDirection ? "正转 / Forward" : "反转 / Reverse");
    }
    lastDebounceTime = millis(); // 更新防抖时间戳 / Update debounce timestamp
  }
  lastDirectionButtonState = currentDirectionButtonState;
}

// 处理电机启动逻辑 / Handle motor start logic
void startMotor() {
  if (motorEnabled) {
    motorStartTime = millis(); // 记录启动时间 / Record start time
    digitalWrite(ENABLE_PIN, LOW); // 启动电机 / Enable motor
    Serial.printf("[%lu] 电机启动 / Motor started\n", getTimestamp());
  }
}

// 修改电机按钮逻辑，增加方向引脚判断 / Modify motor button logic to include direction pin check
void handleMotorButton() {
    // 读取按钮状态 / Read button state
    bool currentMotorButtonState = digitalRead(MOTOR_BUTTON_PIN);
    bool currentDirectionButtonState = digitalRead(BUTTON_DIRECTION_PIN);

    // 检测按钮状态变化并处理防抖 / Detect button state change and handle debounce
    if (currentMotorButtonState != lastMotorButtonState) {
        lastMotorButtonDebounceTime = millis(); // 更新防抖时间戳 / Update debounce timestamp
    }

    // 如果按钮状态稳定超过防抖延迟 / If button state is stable beyond debounce delay
    if ((millis() - lastMotorButtonDebounceTime) > motorButtonDebounceDelay) {
        if (currentMotorButtonState == LOW) { // 如果按钮被按下 / If button is pressed
            updateMotorActivity(); // 更新电机活动时间戳 / Update motor activity timestamp
            if (motorEnabled) {
                // 如果电机已开启，检查方向引脚状态 / If motor is enabled, check direction pin state
                if (currentDirectionButtonState == HIGH) {
                    motorEnabled = false;
                    digitalWrite(ENABLE_PIN, HIGH); // 禁用电机 / Disable motor
                    Serial.printf("[%lu] 电机已关闭（通过按钮） / Motor disabled (via button)\n", millis());
                } else {
                    // 限位触发，准备反转运行 / Limit triggered, prepare for reverse
                    motorDirection = !motorDirection; // 切换电机方向 / Toggle motor direction
                    stepDir = motorDirection;
                    digitalWrite(DIR_PIN, stepDir ? HIGH : LOW); // 设置方向引脚 / Set direction pin
                    Serial.printf("[%lu] 限位触发，电机方向已切换为: %s / Limit triggered, motor direction toggled to: %s\n",
                                  millis(), motorDirection ? "正转 / Forward" : "反转 / Reverse",
                                  motorDirection ? "正转 / Forward" : "反转 / Reverse");
                }
            } else {
                // 如果电机关闭，启动电机 / If motor is disabled, enable the motor
                motorEnabled = true;
                motorStartTime = millis(); // 记录启动时间 / Record start time
                digitalWrite(ENABLE_PIN, LOW); // 启动电机 / Enable motor
                Serial.printf("[%lu] 电机已开启（通过按钮） / Motor enabled (via button)\n", millis());
            }
        }
    }

    // 更新按钮状态 / Update button state
    lastMotorButtonState = currentMotorButtonState;
}

// 处理电机运行时长逻辑 / Handle motor run duration logic
void handleMotorRunDuration() {
  if (motorEnabled && (millis() - motorStartTime >= motorRunDuration)) {
    motorEnabled = false;
    digitalWrite(ENABLE_PIN, HIGH); // 停止电机 / Disable motor
    Serial.printf("[%lu] 电机运行时间到，已停止 / Motor run duration elapsed, stopped\n", millis());
  }
}

// 处理MQTT消息：电机控制 / Handle MQTT message: motor control
void handleMQTTMotorControl(String message) {
  Serial.printf("[%lu] 处理MQTT电机控制消息: %s / Handling MQTT motor control message: %s\n", millis(), message.c_str(), message.c_str());
  updateMotorActivity(); // 更新电机活动时间戳 / Update motor activity timestamp
  if (message == "on") {
    motorEnabled = true;
    digitalWrite(ENABLE_PIN, LOW); // 使能电机 / Enable motor
    client.publish(mqtt_topic_status_report, "Motor On"); // 上报状态 / Report status
    Serial.printf("[%lu] 电机已开启（通过MQTT） / Motor enabled (via MQTT)\n", millis());
  } else if (message == "off") {
    motorEnabled = false;
    digitalWrite(ENABLE_PIN, HIGH); // 禁用电机 / Disable motor
    client.publish(mqtt_topic_status_report, "Motor Off"); // 上报状态 / Report status
    Serial.printf("[%lu] 电机已关闭（通过MQTT） / Motor disabled (via MQTT)\n", millis());
  } else if (message == "forward") {
    motorDirection = true;
    stepDir = motorDirection;
    digitalWrite(DIR_PIN, stepDir ? HIGH : LOW); // 设置为正转 / Set to forward
    client.publish(mqtt_topic_status_report, "Motor Forward"); // 上报状态 / Report status
    Serial.printf("[%lu] 电机正转（通过MQTT） / Motor forward (via MQTT)\n", millis());
  } else if (message == "reverse") {
    motorDirection = false;
    stepDir = motorDirection;
    digitalWrite(DIR_PIN, stepDir ? HIGH : LOW); // 设置为反转 / Set to reverse
    client.publish(mqtt_topic_status_report, "Motor Reverse"); // 上报状态 / Report status
    Serial.printf("[%lu] 电机反转（通过MQTT） / Motor reverse (via MQTT)\n", millis());
  } else {
    Serial.printf("[%lu] 未知的电机控制命令（通过MQTT）: %s / Unknown motor control command (via MQTT): %s\n", millis(), message.c_str(), message.c_str());
  }
}

// 处理Web请求：主页 / Handle web request: root
void handleRoot() {
  String html = MAIN_HTML;
  html.replace("%MICROSTEP_OPTIONS%", MICROSTEP_OPTIONS_HTML);
  server.send(200, "text/html", html);
}

// 处理Web请求：开启电机 / Handle web request: motor on
void handleMotorOn() {
  if (!motorEnabled) {
    motorEnabled = true;
    motorStartTime = millis(); // 记录启动时间 / Record start time
    digitalWrite(ENABLE_PIN, LOW); // 启动电机 / Enable motor
    updateMotorActivity(); // 更新电机活动时间戳 / Update motor activity timestamp
    Serial.printf("[%lu] 电机已开启（通过网页） / Motor enabled (via web)\n", millis());
  }
  server.sendHeader("Content-Type", "text/plain; charset=utf-8");
  server.send(200, "text/plain", "电机已开启 / Motor enabled");
}

// 处理Web请求：关闭电机 / Handle web request: motor off
void handleMotorOff() {
  if (motorEnabled) {
    motorEnabled = false;
    digitalWrite(ENABLE_PIN, HIGH); // 禁用电机 / Disable motor
    Serial.printf("[%lu] 电机已关闭（通过网页） / Motor disabled (via web)\n", millis());
  }
  server.sendHeader("Content-Type", "text/plain; charset=utf-8");
  server.send(200, "text/plain", "电机已关闭 / Motor disabled");
}

// 处理Web请求：切换电机方向 / Handle web request: toggle motor direction
void handleMotorDirection() {
  motorDirection = !motorDirection;
  stepDir = motorDirection;
  digitalWrite(DIR_PIN, stepDir ? HIGH : LOW); // 设置电机方向 / Set motor direction
  Serial.printf("[%lu] 电机方向已切换为: %s（通过网页） / Motor direction toggled to: %s (via web)\n", millis(), motorDirection ? "正转 / Forward" : "反转 / Reverse", motorDirection ? "正转 / Forward" : "反转 / Reverse");
  server.sendHeader("Content-Type", "text/plain; charset=utf-8");
  server.send(200, "text/plain", motorDirection ? "电机正转 / Motor forward" : "电机反转 / Motor reverse");
}

// 处理API请求：电机控制 / Handle API request: motor control
void handleMotorAPI() {
  if (server.hasArg("command")) {
    String command = server.arg("command");
    Serial.printf("[%lu] 收到API请求，命令: %s / Received API request, command: %s\n", getTimestamp(), command.c_str(), command.c_str());
    updateMotorActivity(); // 更新电机活动时间戳 / Update motor activity timestamp
    if (command == "on") {
      motorEnabled = true;
      digitalWrite(ENABLE_PIN, LOW); // 使能电机 / Enable motor
      server.send(200, "text/plain", "电机已开启 / Motor enabled");
      Serial.printf("[%lu] 电机已开启（通过API） / Motor enabled (via API)\n", getTimestamp());
    } else if (command == "off") {
      motorEnabled = false;
      digitalWrite(ENABLE_PIN, HIGH); // 禁用电机 / Disable motor
      server.send(200, "text/plain", "电机已关闭 / Motor disabled");
      Serial.printf("[%lu] 电机已关闭（通过API） / Motor disabled (via API)\n", getTimestamp());
    } else if (command == "forward") {
      motorDirection = true;
      stepDir = motorDirection;
      digitalWrite(DIR_PIN, stepDir ? HIGH : LOW); // 设置为正转 / Set to forward
      server.send(200, "text/plain", "电机正转 / Motor forward");
      Serial.printf("[%lu] 电机正转（通过API） / Motor forward (via API)\n", getTimestamp());
    } else if (command == "reverse") {
      motorDirection = false;
      stepDir = motorDirection;
      digitalWrite(DIR_PIN, stepDir ? HIGH : LOW); // 设置为反转 / Set to reverse
      server.send(200, "text/plain", "电机反转 / Motor reverse");
      Serial.printf("[%lu] 电机反转（通过API） / Motor reverse (via API)\n", getTimestamp());
    } else {
      server.send(400, "text/plain", "未知命令 / Unknown command");
      Serial.printf("[%lu] 收到未知命令（通过API） / Unknown command received (via API)\n", getTimestamp());
    }
  } else {
    server.send(400, "text/plain", "缺少命令参数 / Missing command parameter");
    Serial.printf("[%lu] API请求缺少命令参数 / API request missing command parameter\n", getTimestamp());
  }
}

// 处理Web请求：获取版本信息 / Handle web request: get version information
void handleVersionInfo() {
  String versionInfo = String("固件版本: ") + FIRMWARE_VERSION + " / Firmware Version: " + FIRMWARE_VERSION;
  server.send(200, "text/plain", versionInfo);
}

// 处理OTA升级界面 / Handle OTA upgrade page
void handleOTA() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="UTF-8">
      <title>OTA 升级</title>
      <style>
        body { font-family: Arial, sans-serif; text-align: center; margin: 20px; }
        h1 { color: #333; }
        p { font-size: 16px; }
        input[type="file"], button, input[type="text"] { padding: 10px; margin: 10px; font-size: 16px; }
        button { background-color: #4CAF50; color: white; border: none; cursor: pointer; }
        button:hover { background-color: #45a049; }
        input[type="text"] { width: 300px; }
      </style>
    </head>
    <body>
      <h1>OTA 升级</h1>
      <p>通过以下方式上传固件文件或指定远程地址进行升级。</p>
      <h2>上传固件文件</h2>
      <form method="POST" action="/ota/upload" enctype="multipart/form-data">
        <input type="file" name="firmware">
        <button type="submit">上传并升级</button>
      </form>
      <h2>远程升级</h2>
      <form method="POST" action="/ota/remote">
        <input type="text" name="url" placeholder="输入远程固件地址" required>
        <button type="submit">开始远程升级</button>
      </form>
      <button onclick="location.href='/'">返回主页面</button>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// 处理OTA文件上传 / Handle OTA file upload
void handleOTAUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("开始上传固件: %s\n", upload.filename.c_str());
    size_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000; // 获取最大可用空间 / Get maximum available space
    if (!Update.begin(maxSketchSpace)) { // 初始化OTA更新 / Initialize OTA update
      Serial.printf("OTA初始化失败，错误代码: %d\n", Update.getError()); // 输出错误代码 / Print error code
      server.send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
          <meta charset="UTF-8">
          <script>
            alert('OTA初始化失败 / OTA initialization failed');
            window.location.href = '/ota';
          </script>
        </head>
        <body></body>
        </html>
      )rawliteral");
      return;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Serial.printf("OTA写入失败，错误代码: %d\n", Update.getError()); // 输出错误代码 / Print error code
      server.send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
          <meta charset="UTF-8">
          <script>
            alert('OTA写入失败 / OTA write failed');
            window.location.href = '/ota';
          </script>
        </head>
        <body></body>
        </html>
      )rawliteral");
      return;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { // 完成OTA更新 / Finish OTA update
      Serial.println("OTA更新成功 / OTA update successful");
      // 恢复为原先的弹窗+网页
      server.send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
          <meta charset="UTF-8">
          <script>
            alert('OTA更新成功，设备将在5秒后重启 / OTA update successful, device will restart in 5 seconds');
            setTimeout(() => { window.location.href = '/'; }, 5000);
          </script>
        </head>
        <body>
          <h1>OTA更新成功 / OTA Update Successful</h1>
          <p>设备将在5秒后重启 / The device will restart in 5 seconds.</p>
        </body>
        </html>
      )rawliteral");
      delay(5000); // 延迟5秒以显示成功信息 / Delay 5 seconds to show success message
      ESP.restart(); // 重启设备 / Restart device
    } else {
      Serial.printf("OTA更新失败，错误代码: %d\n", Update.getError()); // 输出错误代码 / Print error code
      server.send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
          <meta charset="UTF-8">
          <script>
            alert('OTA更新失败 / OTA update failed');
            window.location.href = '/ota';
          </script>
        </head>
        <body></body>
        </html>
      )rawliteral");
    }
  }
}

// 处理远程OTA升级 / Handle remote OTA upgrade
void handleOTARemote() {
  if (server.hasArg("url")) {
    String url = server.arg("url");
    Serial.printf("开始远程OTA升级，地址: %s\n", url.c_str());
    WiFiClient client;
    HTTPClient http;

    if (http.begin(client, url)) { // 开始下载固件 / Start downloading firmware
      int httpCode = http.GET();
      if (httpCode == HTTP_CODE_OK) {
        size_t contentLength = static_cast<size_t>(http.getSize());
        if (contentLength > 0) {
          if (Update.begin(contentLength)) { // 初始化OTA更新 / Initialize OTA update
            WiFiClient* stream = http.getStreamPtr();
            size_t written = Update.writeStream(*stream);
            if (written == contentLength && Update.end()) {
              Serial.println("远程OTA更新成功 / Remote OTA update successful");
              server.send(200, "text/html", R"rawliteral(
                <!DOCTYPE html>
                <html>
                <head>
                  <meta charset="UTF-8">
                  <script>
                    alert('远程OTA更新成功，设备即将重启 / Remote OTA update successful, device will restart');
                    window.location.href = '/';
                  </script>
                </head>
                <body></body>
                </html>
              )rawliteral");
              ESP.restart(); // 重启设备 / Restart device
            } else {
              Serial.printf("远程OTA更新失败，错误代码: %d\n", Update.getError());
              server.send(200, "text/html", R"rawliteral(
                <!DOCTYPE html>
                <html>
                <head>
                  <meta charset="UTF-8">
                  <script>
                    alert('远程OTA更新失败 / Remote OTA update failed');
                    window.location.href = '/ota';
                  </script>
                </head>
                <body></body>
                </html>
              )rawliteral");
            }
          } else {
            Serial.printf("OTA初始化失败，错误代码: %d\n", Update.getError());
            server.send(200, "text/html", R"rawliteral(
              <!DOCTYPE html>
              <html>
              <head>
                <meta charset="UTF-8">
                <script>
                  alert('OTA初始化失败 / OTA initialization failed');
                  window.location.href = '/ota';
                </script>
              </head>
              <body></body>
              </html>
            )rawliteral");
          }
        } else {
          Serial.println("远程固件大小无效 / Invalid firmware size");
          server.send(200, "text/html", R"rawliteral(
            <!DOCTYPE html>
            <html>
            <head>
              <meta charset="UTF-8">
              <script>
                alert('远程固件大小无效 / Invalid firmware size');
                window.location.href = '/ota';
              </script>
            </head>
            <body></body>
            </html>
          )rawliteral");
        }
      } else {
        Serial.printf("HTTP请求失败，状态码: %d\n", httpCode);
        server.send(200, "text/html", R"rawliteral(
          <!DOCTYPE html>
          <html>
          <head>
            <meta charset="UTF-8">
            <script>
              alert('HTTP请求失败 / HTTP request failed');
              window.location.href = '/ota';
            </script>
          </head>
          <body></body>
          </html>
        )rawliteral");
      }
      http.end();
    } else {
      Serial.println("无法连接到远程地址 / Unable to connect to remote URL");
      server.send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
          <meta charset="UTF-8">
          <script>
            alert('无法连接到远程地址 / Unable to connect to remote URL');
            window.location.href = '/ota';
          </script>
        </head>
        <body></body>
        </html>
      )rawliteral");
    }
  } else {
    server.send(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
        <meta charset="UTF-8">
        <script>
          alert('缺少远程地址参数 / Missing remote URL parameter');
          window.location.href = '/ota';
      </script>
      </head>
      <body></body>
      </html>
    )rawliteral");
  }
}

// 处理启用或禁用MQTT控制的请求 / Handle enable or disable MQTT control
void handleToggleMQTTControl() {
  if (server.hasArg("enable")) {
    String enable = server.arg("enable");
    if (enable == "true") {
      mqttControlEnabled = true;
      client.setServer(mqtt_server, 1883); // 设置MQTT服务器地址 / Set MQTT server address
      Serial.println("MQTT控制已启用 / MQTT control enabled");
      server.send(200, "text/plain; charset=utf-8", "MQTT控制已启用 / MQTT control enabled");
    } else if (enable == "false") {
      mqttControlEnabled = false;
      client.unsubscribe(mqtt_topic_motor_control); // 取消订阅主题 / Unsubscribe from topic
      client.disconnect(); // 禁用时断开MQTT连接 / Disconnect MQTT when disabled
      Serial.println("MQTT控制已禁用 / MQTT control disabled");
      server.send(200, "text/plain; charset=utf-8", "MQTT控制已禁用 / MQTT control disabled");
    } else {
      server.send(400, "text/plain; charset=utf-8", "无效参数 / Invalid parameter");
      Serial.println("无效的MQTT控制参数 / Invalid MQTT control parameter");
    }
  } else {
    server.send(400, "text/plain; charset=utf-8", "缺少参数 / Missing parameter");
    Serial.println("缺少MQTT控制参数 / Missing MQTT control parameter");
  }
}

// 处理设备信息请求 / Handle device info request
void handleDeviceInfo() {
  String deviceInfo = "{";
  deviceInfo += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  deviceInfo += "\"mac\":\"" + WiFi.macAddress() + "\",";
  deviceInfo += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
  deviceInfo += "\"onlineClients\":" + String(clients.size());
  deviceInfo += "}";
  server.send(200, "application/json", deviceInfo);
}

// 处理设置电机启动时长的网页请求 / Handle web request to set motor run duration
void handleSetMotorRunDuration() {
  if (server.hasArg("duration")) {
    int duration = server.arg("duration").toInt();
    if (duration >= 1 && duration <= 1800) { // 范围：1秒到30分钟 / Range: 1 second to 30 minutes
      motorRunDuration = duration * 1000; // 转换为毫秒 / Convert to milliseconds
      server.send(200, "text/plain; charset=utf-8", "电机启动时长已更新 / Motor run duration updated");
      Serial.printf("电机启动时长设置为: %d 秒 / Motor run duration set to: %d seconds\n", duration, duration);
    } else {
      server.send(400, "text/plain; charset=utf-8", "无效的时长，范围为1到1800秒 / Invalid duration, range is 1 to 1800 seconds");
      Serial.println("设置电机启动时长失败：无效的时长 / Failed to set motor run duration: Invalid duration");
    }
  } else {
    server.send(400, "text/plain; charset=utf-8", "缺少时长参数 / Missing duration parameter");
    Serial.println("设置电机启动时长失败：缺少时长参数 / Failed to set motor run duration: Missing duration parameter");
  }
}

// 处理控制端信息页面请求 / Handle client info page request
void handleClientsPage() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="UTF-8">
      <title>控制端设备信息</title>
      <style>
        body { font-family: Arial, sans-serif; text-align: center; margin: 20px; }
        table { margin: 20px auto; border-collapse: collapse; width: 80%; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }
        th { background-color: #4CAF50; color: white; }
        tr:nth-child(even) { background-color: #f2f2f2; }
        tr:hover { background-color: #ddd; }
        .back-button {
          margin-top: 20px;
          padding: 10px 20px;
          font-size: 16px;
          color: white;
          background-color: #4CAF50;
          border: none;
          border-radius: 5px;
          box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
          cursor: pointer;
          transition: background-color 0.3s, box-shadow 0.3s;
        }
        .back-button:hover {
          background-color: #45a049;
          box-shadow: 0 6px 8px rgba(0, 0, 0, 0.15);
        }
      </style>
    </head>
    <body>
      <h1>控制端设备信息</h1>
      <table>
        <tr>
          <th>设备名称</th>
          <th>MAC 地址</th>
        </tr>
  )rawliteral";

  // 遍历控制端信息并生成表格行 / Iterate over clients and generate table rows
  for (const auto& client : clients) {
    html += "<tr><td>" + client.second + "</td><td>" + client.first + "</td></tr>";
  }

  html += R"rawliteral(
      </table>
      <button class="back-button" onclick="location.href='/'">返回主页面</button>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// 处理设置控制端名称的请求 / Handle set client name request
void handleSetClientName() {
  if (server.hasArg("mac") && server.hasArg("name")) {
    String mac = server.arg("mac");
    String name = server.arg("name");
    clients[mac] = name; // 更新或添加控制端信息 / Update or add client info
    server.send(200, "text/plain", "控制端名称已更新 / Client name updated");
    Serial.printf("控制端名称已更新: MAC=%s, 名称=%s\n", mac.c_str(), name.c_str());
  } else {
    server.send(400, "text/plain", "缺少参数 / Missing parameters");
  }
}

// 更新控制端在线状态 / Update client online status
void updateClientOnlineStatus(String mac) {
  if (clients.find(mac) == clients.end()) {
    clients[mac] = "默认名称"; // 如果未设置名称，使用默认名称 / Use default name if not set
  }
  Serial.printf("控制端在线: MAC=%s\n", mac.c_str());
}

// 处理重新配网请求 / Handle reset WiFi request
void handleResetWiFi() {
  Serial.println("收到重新配网请求，准备进入智能配网模式 / Received reset WiFi request, preparing to enter smart configuration mode");
  server.send(200, "text/plain; charset=utf-8", "设备正在重新进入配网模式，请稍候... / Device is restarting to enter configuration mode, please wait...");
  delay(1000); // 延迟以确保响应发送完成 / Delay to ensure the response is sent
  WiFi.disconnect(); // 断开WiFi连接 / Disconnect WiFi
  Serial.println("WiFi已断开，设备即将重启 / WiFi disconnected, device will restart");
  ESP.restart(); // 重启设备以进入智能配网模式 / Restart device to enter smart configuration mode
}

// 新增API：设置细分模式
void handleSetMicrostep() {
    if (server.hasArg("mode")) {
        int mode = server.arg("mode").toInt();
        if (mode == MICROSTEP_FULL || mode == MICROSTEP_8 || mode == MICROSTEP_16 || mode == MICROSTEP_32) {
            setMicrostepMode(mode);
            server.send(200, "text/plain; charset=utf-8", "细分模式已切换");
        } else {
            server.send(400, "text/plain; charset=utf-8", "无效细分模式");
        }
    } else {
        server.send(400, "text/plain; charset=utf-8", "缺少参数");
    }
}

// 新增API：获取当前细分模式
void handleGetMicrostep() {
    String json = "{\"mode\":" + String(currentMicrostep) + "}";
    server.send(200, "application/json", json);
}

// 新增单步运行接口，便于调试和外部调用
void handleStepOnce() {
    stepMotorOnce();
    server.send(200, "text/plain; charset=utf-8", "单步运行已执行 / Step motor once executed");
}

// 新增API接口：单步运行（API风格，支持GET/POST）
void handleApiStepOnce() {
    stepMotorOnce();
    server.send(200, "application/json", "{\"result\":true,\"msg\":\"step once ok\"}");
}



