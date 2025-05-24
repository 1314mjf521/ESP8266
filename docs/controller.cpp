#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>

// 按钮引脚定义 / Button pin definitions
#define BUTTON_FORWARD_PIN D1 // 电机正向运动按钮引脚 / Motor forward button pin
#define BUTTON_REVERSE_PIN D2 // 电机反向运动按钮引脚 / Motor reverse button pin

// LED 引脚定义 / LED pin definition
#define LED_PIN D4 // 使用板载 LED / Use onboard LED

// LED 状态变量 / LED state variables
bool ledState = LOW;
unsigned long lastLedToggleTime = 0;
unsigned long ledBlinkInterval = 500; // 默认 LED 频闪间隔（毫秒） / Default LED blink interval (milliseconds)

// MQTT 配置 / MQTT configuration
#define EEPROM_SIZE 512
#define MQTT_ADDRESS_OFFSET 0
#define MQTT_ADDRESS_MAX_LENGTH 100

char mqtt_server[MQTT_ADDRESS_MAX_LENGTH] = ""; // MQTT 服务端地址 / MQTT server address
const char* mqtt_topic_motor_control = "motor/control"; // 电机控制主题 / Motor control topic
const char* mqtt_topic_register = "controller/register"; // 控制端注册主题 / Controller register topic

WiFiClient espClient;
PubSubClient client(espClient);
ESP8266WebServer server(80); // 创建 Web 服务器实例 / Create web server instance

// 按钮状态变量 / Button state variables
bool lastForwardButtonState = HIGH;
bool lastReverseButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 防抖延迟（毫秒） / Debounce delay (milliseconds)

// WiFi 连接失败计数器 / WiFi connection failure counter
int wifiConnectFailures = 0;

// 更新 LED 状态的函数 / Function to update LED state
void updateLEDState() {
  unsigned long currentMillis = millis();

  // WiFi 未连接或断开时爆闪 / Rapid blinking when WiFi is disconnected
  if (WiFi.status() != WL_CONNECTED) {
    ledBlinkInterval = 100; // 高频闪烁 / High-frequency blinking
  }
  // WiFi 已连接但 MQTT 未配置时频闪 / Blinking when WiFi is connected but MQTT is not configured
  else if (mqtt_server[0] == '\0') {
    ledBlinkInterval = 500; // 频闪 / Regular blinking
  }
  // WiFi 和 MQTT 都已连接时常亮 / LED on when both WiFi and MQTT are connected
  else if (client.connected()) {
    digitalWrite(LED_PIN, HIGH); // 常亮 / LED on
    return;
  }
  // WiFi 已连接但 MQTT 连接失败时周期闪烁 / Periodic blinking when WiFi is connected but MQTT fails
  else {
    ledBlinkInterval = 1000; // 周期闪烁 / Periodic blinking
  }

  // 控制 LED 闪烁逻辑 / Handle LED blinking logic
  if (currentMillis - lastLedToggleTime >= ledBlinkInterval) {
    lastLedToggleTime = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

// 修改 setupWiFi 函数，确保 WiFi 配置完成后始终可以访问网页 / Ensure web access after WiFi configuration
void setupWiFi() {
  WiFiManager wifiManager;
  wifiManager.setTimeout(180); // 设置智能配网超时时间为 180 秒 / Set smart configuration timeout to 180 seconds

  while (wifiConnectFailures < 5) {
    if (wifiManager.autoConnect("ESP8266_Controller")) { // 设置配网热点名称 / Set WiFi configuration hotspot name
      Serial.println("WiFi 已连接 / WiFi connected");
      Serial.print("IP 地址: / IP Address: ");
      Serial.println(WiFi.localIP());
      return;
    } else {
      wifiConnectFailures++;
      Serial.printf("WiFi 连接失败，第 %d 次尝试 / WiFi connection failed, attempt %d\n", wifiConnectFailures, wifiConnectFailures);
      delay(10000); // 每次失败后等待 10 秒 / Wait 10 seconds after each failure
    }
  }

  // 超过 5 次失败后重新进入智能配网模式 / Enter smart configuration mode after 5 failures
  Serial.println("WiFi 连接失败超过 5 次，进入智能配网模式 / WiFi connection failed more than 5 times, entering smart configuration mode");
  wifiManager.startConfigPortal("ESP8266_Controller");
}

// 加载保存的 MQTT 地址 / Load saved MQTT address
void loadMQTTAddress() {
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < MQTT_ADDRESS_MAX_LENGTH; i++) {
    mqtt_server[i] = EEPROM.read(MQTT_ADDRESS_OFFSET + i);
    if (mqtt_server[i] == '\0') break;
  }
  if (mqtt_server[0] == '\0') {
    Serial.println("未配置 MQTT 地址 / MQTT address not configured");
  } else {
    Serial.println("加载的 MQTT 地址: " + String(mqtt_server));
  }
}

// 保存 MQTT 地址到 EEPROM / Save MQTT address to EEPROM
void saveMQTTAddress(const char* address) {
  for (int i = 0; i < MQTT_ADDRESS_MAX_LENGTH; i++) {
    EEPROM.write(MQTT_ADDRESS_OFFSET + i, address[i]);
    if (address[i] == '\0') break;
  }
  EEPROM.commit();
  Serial.println("保存的 MQTT 地址: " + String(address));
}

// MQTT 回调函数 / MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // 控制端无需处理消息，仅作为发布者 / Controller does not process messages, only publishes
}

// MQTT 客户端 ID 动态生成 / Generate dynamic MQTT client ID
String generateMQTTClientID() {
  String macAddress = WiFi.macAddress();
  macAddress.replace(":", ""); // 移除冒号 / Remove colons
  return "ESP8266Controller_" + macAddress; // 拼接唯一客户端 ID / Concatenate unique client ID
}

// 修改 connectToMQTT 函数，确保 MQTT 连接失败不会阻塞其他功能 / Ensure MQTT connection failure does not block other functionality
void connectToMQTT() {
  if (mqtt_server[0] == '\0') {
    Serial.println("MQTT 地址未配置，跳过连接 / MQTT address not configured, skipping connection");
    return;
  }

  if (!client.connected()) {
    String clientID = generateMQTTClientID(); // 动态生成客户端 ID / Dynamically generate client ID
    Serial.print("连接到 MQTT 服务器中... / Connecting to MQTT server...");
    if (client.connect(clientID.c_str())) {
      Serial.println("已连接 / Connected");
    } else {
      Serial.print("连接失败，状态码: / Connection failed, state: ");
      Serial.println(client.state());
      delay(5000); // 5 秒后重试 / Retry after 5 seconds
    }
  }
}

// 修改 handleButtons 函数，分别处理正向和反向按钮 / Handle forward and reverse buttons separately
void handleButtons() {
  // 读取按钮状态 / Read button states
  bool currentForwardButtonState = digitalRead(BUTTON_FORWARD_PIN);
  bool currentReverseButtonState = digitalRead(BUTTON_REVERSE_PIN);

  // 处理正向按钮 / Handle forward button
  if (currentForwardButtonState != lastForwardButtonState) {
    if (currentForwardButtonState == LOW) { // 按钮被按下 / Button pressed
      client.publish(mqtt_topic_motor_control, "forward"); // 发布正向运动命令 / Publish forward command
      Serial.println("正向按钮按下，发布正向运动命令 / Forward button pressed, published forward command");
    }
    lastDebounceTime = millis();
  }
  lastForwardButtonState = currentForwardButtonState;

  // 处理反向按钮 / Handle reverse button
  if (currentReverseButtonState != lastReverseButtonState) {
    if (currentReverseButtonState == LOW) { // 按钮被按下 / Button pressed
      client.publish(mqtt_topic_motor_control, "reverse"); // 发布反向运动命令 / Publish reverse command
      Serial.println("反向按钮按下，发布反向运动命令 / Reverse button pressed, published reverse command");
    }
    lastDebounceTime = millis();
  }
  lastReverseButtonState = currentReverseButtonState;
}

// 处理网页请求：显示 MQTT 配置页面 / Handle web request: show MQTT configuration page
void handleMQTTConfigPage() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="UTF-8">
      <title>MQTT 配置</title>
      <style>
        body { font-family: Arial, sans-serif; text-align: center; margin: 20px; }
        h1 { color: #333; }
        input[type="text"], button { padding: 10px; margin: 10px; font-size: 16px; }
        button { background-color: #4CAF50; color: white; border: none; cursor: pointer; }
        button:hover { background-color: #45a049; }
        input[type="text"] { width: 300px; }
      </style>
    </head>
    <body>
      <h1>MQTT 配置</h1>
      <form action="/set_mqtt" method="GET">
        <input type="text" name="address" placeholder="输入 MQTT 服务器地址" required>
        <button type="submit">保存</button>
      </form>
      <form action="/unbind_mqtt" method="GET">
        <button type="submit">解绑</button>
      </form>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// 修改 handleUnbindMQTTAddress 函数，确保解绑后仍可访问网页 / Ensure web access after unbinding MQTT info
void handleUnbindMQTTAddress() {
  memset(mqtt_server, 0, MQTT_ADDRESS_MAX_LENGTH); // 清空 MQTT 地址 / Clear MQTT address
  saveMQTTAddress(mqtt_server); // 保存空地址到 EEPROM / Save empty address to EEPROM

  // 立即返回成功响应 / Immediately return success response
  server.send(200, "text/html", R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="UTF-8">
      <script>
        alert('MQTT 地址已解绑 / MQTT address unbound');
        window.location.href = '/';
      </script>
    </head>
    <body></body>
    </html>
  )rawliteral");

  Serial.println("MQTT 地址已解绑 / MQTT address unbound");

  // 在后台断开 MQTT 连接 / Disconnect MQTT in the background
  if (client.connected()) {
    client.disconnect();
  }
}

// 修改 handleSetMQTTAddress 函数，确保设置完 MQTT 信息后仍可访问网页 / Ensure web access after setting MQTT info
void handleSetMQTTAddress() {
  if (server.hasArg("address")) { // 检查是否提供了地址参数 / Check if address parameter is provided
    String address = server.arg("address");
    if (address.length() < MQTT_ADDRESS_MAX_LENGTH) {
      address.toCharArray(mqtt_server, MQTT_ADDRESS_MAX_LENGTH); // 将地址转换为字符数组 / Convert address to char array
      saveMQTTAddress(mqtt_server); // 保存地址到 EEPROM / Save address to EEPROM
      client.setServer(mqtt_server, 1883); // 更新 MQTT 服务器地址 / Update MQTT server address

      // 立即返回成功响应 / Immediately return success response
      server.send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
          <meta charset="UTF-8">
          <script>
            alert('MQTT 地址已更新 / MQTT address updated');
            window.location.href = '/';
          </script>
        </head>
        <body></body>
        </html>
      )rawliteral");

      Serial.println("MQTT 地址已更新为: " + address);

      // 在后台尝试重新连接 MQTT / Attempt to reconnect to MQTT in the background
      if (client.connected()) {
        client.disconnect();
      }
      connectToMQTT();
    } else {
      server.send(400, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
          <meta charset="UTF-8">
          <script>
            alert('地址过长 / Address too long');
            window.location.href = '/';
          </script>
        </head>
        <body></body>
        </html>
      )rawliteral");
      Serial.println("设置 MQTT 地址失败：地址过长 / Failed to set MQTT address: Address too long");
    }
  } else {
    server.send(400, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
        <meta charset="UTF-8">
        <script>
          alert('缺少地址参数 / Missing address parameter');
          window.location.href = '/';
        </script>
      </head>
      <body></body>
      </html>
    )rawliteral");
    Serial.println("设置 MQTT 地址失败：缺少地址参数 / Failed to set MQTT address: Missing address parameter");
  }
}

// 修改 setup 函数，确保无论 MQTT 是否连接成功，网页始终可访问 / Ensure web access regardless of MQTT connection status
void setup() {
  Serial.begin(115200);

  // 初始化 LED 引脚 / Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 默认关闭 LED / Default LED off

  // 初始化按钮引脚 / Initialize button pins
  pinMode(BUTTON_FORWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_REVERSE_PIN, INPUT_PULLUP);

  // 初始化 WiFi / Initialize WiFi
  setupWiFi();

  // 加载 MQTT 地址 / Load MQTT address
  loadMQTTAddress();

  // 初始化 Web 服务器 / Initialize web server
  server.on("/", handleMQTTConfigPage); // 显示 MQTT 配置页面 / Show MQTT configuration page
  server.on("/set_mqtt", handleSetMQTTAddress); // 设置 MQTT 地址 / Set MQTT address
  server.on("/unbind_mqtt", handleUnbindMQTTAddress); // 解绑 MQTT 地址 / Unbind MQTT address
  server.begin();
  Serial.println("Web 服务器已启动 / Web server started");

  // 初始化 MQTT 客户端，但不阻塞网页访问 / Initialize MQTT client without blocking web access
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

void loop() {
  // 更新 LED 状态 / Update LED state
  updateLEDState();

  // 始终处理网页请求 / Always handle web requests
  server.handleClient();

  // 如果 MQTT 地址已配置，尝试连接 MQTT / If MQTT address is configured, attempt to connect to MQTT
  if (mqtt_server[0] != '\0') {
    if (!client.connected()) {
      connectToMQTT(); // 确保 MQTT 连接 / Ensure MQTT connection
    }
    client.loop(); // 处理 MQTT 消息 / Handle MQTT messages
  }

  handleButtons(); // 处理按钮逻辑 / Handle button logic
}
