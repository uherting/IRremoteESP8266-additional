#define IRRX_PIN 13 // D7
#define IRTX_PIN 15 // D8

#include <pgmspace.h>
#include <FS.h>
#include "ESPWebMQTT.h"
#include "Date.h"
#include "Schedule.h"
#include "RTCmem.h"
#include <IRremoteESP8266.h>
#ifdef IRRX_PIN
#include <IRrecv.h>
#endif
#include <IRsend.h>

const int8_t MAX_SCHEDULES = 10; // Количество элементов расписания

const char overSSID[] PROGMEM = "IRblaster_"; // Префикс имени точки доступа по умолчанию
const char overMQTTClient[] PROGMEM = "IRblaster_"; // Префикс имени MQTT-клиента по умолчанию

const char pathRemote[] PROGMEM = "/remote"; // Путь до страницы настройки параметров кнопок ДУ
const char pathGetRemote[] PROGMEM = "/getremote"; // Путь до страницы, возвращающей JSON-пакет кнопки ДУ
const char pathSetRemote[] PROGMEM = "/setremote"; // Путь до страницы изменения кнопки ДУ
const char pathRemoteData[] PROGMEM = "/remotedata"; // Путь до страницы, возвращающей JSON-пакет данных о последней нажатой кнопке пульта ДУ
const char pathIRSend[] PROGMEM = "/irsend"; // Путь до страницы отправки ИК-посылок
const char pathSchedulesJs[] PROGMEM = "/schedules.js";
const char pathSchedules[] PROGMEM = "/schedules"; // Путь до страницы настройки параметров расписания
const char pathGetSchedule[] PROGMEM = "/getschedule"; // Путь до страницы, возвращающей JSON-пакет элемента расписания
const char pathSetSchedule[] PROGMEM = "/setschedule"; // Путь до страницы изменения элемента расписания

// Имена параметров для Web-форм
const char paramRemoteBtnName[] PROGMEM = "rembtnname";
const char paramRemoteBtnCode[] PROGMEM = "rembtncode";
const char paramRemoteBtnRepeat[] PROGMEM = "rembtnrepeat";
const char paramRemoteBtnGap[] PROGMEM = "rembtngap";
const char paramSchedulePeriod[] PROGMEM = "period";
const char paramScheduleHour[] PROGMEM = "hour";
const char paramScheduleMinute[] PROGMEM = "minute";
const char paramScheduleSecond[] PROGMEM = "second";
const char paramScheduleWeekdays[] PROGMEM = "weekdays";
const char paramScheduleDay[] PROGMEM = "day";
const char paramScheduleMonth[] PROGMEM = "month";
const char paramScheduleYear[] PROGMEM = "year";
const char paramScheduleIRButton[] PROGMEM = "irbutton";

// Имена JSON-переменных
const char jsonRemoteCode[] PROGMEM = "remotecode";

// Названия топиков для MQTT
const char mqttRemoteBtnTopic[] PROGMEM = "/IRButton";

const char remoteFileName[] PROGMEM = "/IRblaster.dat";

const char strNone[] PROGMEM = "(None)";

class ESPIRBlaster : public ESPWebMQTTBase {
public:
  ESPIRBlaster() : ESPWebMQTTBase() {}

protected:
#ifdef IRRX_PIN
  void cleanup();
#endif

  void setupExtra();
  void loopExtra();

  String getHostName();

  bool readConfig(uint16_t &offset);
  bool writeConfig(uint16_t &offset, bool commit = true);
  void defaultConfig(uint8_t level = 0);

  void setupHttpServer();
  void handleRootPage();
  void handleRemoteConfig(); // Обработчик страницы настройки параметров кнопок ДУ
  void handleGetRemote(); // Обработчик страницы, возвращающей JSON-пакет кнопки ДУ
  void handleSetRemote(); // Обработчик страницы изменения кнопки ДУ
#ifdef IRRX_PIN
  void handleRemoteData(); // Обработчик страницы, возвращающей JSON-пакет данных о последней нажатой кнопке пульта ДУ
#endif
  void handleIRSend(); // Обработчик страницы посылки кода кнопки ДУ
  void handleSchedulesJs();
  void handleSchedulesConfig(); // Обработчик страницы настройки параметров расписания
  void handleGetSchedule(); // Обработчик страницы, возвращающей JSON-пакет элемента расписания
  void handleSetSchedule(); // Обработчик страницы изменения элемента расписания

  String navigator();
  String btnRemoteConfig(); // HTML-код кнопки вызова настройки кнопок ДУ
  String btnSchedulesConfig(); // HTML-код кнопки вызова настройки расписания

  void mqttCallback(char *topic, byte *payload, unsigned int length);
  void mqttResubscribe();

private:
  bool readSchedulesConfig(uint16_t &offset); // Чтение из EEPROM порции параметров расписания
  bool writeSchedulesConfig(uint16_t &offset); // Запись в EEPROM порции параметров расписания

  bool readIRButtons();
  bool writeIRButtons();
  void clearIRButtons();

  static const uint8_t BUTTON_COLS = 3;
  static const uint8_t BUTTON_ROWS = 7;

  static const uint8_t BUTTON_NAME_SIZE = 16;
  static const uint16_t IR_CAPTURE_BUFFER_SIZE = 128;
  static const uint8_t IR_TIMEOUT = 45; // 15

  struct irbutton_t {
    char buttonName[BUTTON_NAME_SIZE];
    union {
      struct {
        uint8_t repeat : 4; // 0..15 = 1..16
        uint16_t gap : 12; // 0..4095 ms
      };
      uint16_t repeatgap;
    };
    uint16_t rawBufLen;
    uint16_t rawBuf[IR_CAPTURE_BUFFER_SIZE];
  } irbuttons[BUTTON_COLS * BUTTON_ROWS];

  void sendButtonCode(uint8_t btn);

#ifdef IRRX_PIN
  bool cloneRemoteCode(decode_results *results);

  uint16_t rawBufLen;
  uint16_t rawBuf[IR_CAPTURE_BUFFER_SIZE];

  IRrecv *irRX;
#endif
  IRsend *irTX;

  Schedule schedules[MAX_SCHEDULES]; // Массив расписания событий
  int8_t scheduleButtons[MAX_SCHEDULES]; // Что делать с реле по срабатыванию события
};

/***
 * ESPIRBlaster class implemenattion
 */

#ifdef IRRX_PIN
void ESPIRBlaster::cleanup() {
  irRX->disableIRIn();

  ESPWebMQTTBase::cleanup();
}
#endif

void ESPIRBlaster::setupExtra() {
  ESPWebMQTTBase::setupExtra();

#ifdef IRRX_PIN
  irRX = new IRrecv(IRRX_PIN, IR_CAPTURE_BUFFER_SIZE, IR_TIMEOUT, true);
  irRX->enableIRIn();
#endif
  irTX = new IRsend(IRTX_PIN);
  irTX->begin();
}

void ESPIRBlaster::loopExtra() {
  ESPWebMQTTBase::loopExtra();

#ifdef IRRX_PIN
  static decode_results results;

  if (irRX->decode(&results)) {
    cloneRemoteCode(&results);
    irRX->resume();
  }
#endif

  uint32_t now = getTime();

  if (now) {
    for (int8_t i = 0; i < MAX_SCHEDULES; ++i) {
      if (schedules[i].period() != Schedule::NONE) {
        if (schedules[i].check(now)) {
          if ((scheduleButtons[i] >= 0) && (scheduleButtons[i] < BUTTON_ROWS * BUTTON_COLS)) {
            logDateTime(now);
            _log->print(F(" schedule \""));
            _log->print(schedules[i]);
            _log->println(F("\" triggered"));
            sendButtonCode(scheduleButtons[i]);
          }
        }
      }
    }
  }
}

String ESPIRBlaster::getHostName() {
  String result;

  result = FPSTR(overSSID);
  result += getBoardId();

  return result;
}

bool ESPIRBlaster::readConfig(uint16_t &offset) {
  if (! ESPWebMQTTBase::readConfig(offset))
    return false;

  uint16_t start = offset;

  if (! readSchedulesConfig(offset)) {
    _log->println(F("Error reading schedules configuration!"));
    defaultConfig(2);
    return false;
  }

  uint8_t crc = crc8EEPROM(start, offset);
  if (readEEPROM(offset) != crc) {
    _log->println(F("CRC mismatch! Use default sketch parameters."));
    defaultConfig(2);
    return false;
  }

  if (! readIRButtons()) {
    _log->println(F("Unable to read IR buttons configuration file!"));
    clearIRButtons();
  }

  return true;
}

bool ESPIRBlaster::writeConfig(uint16_t &offset, bool commit) {
  if (! ESPWebMQTTBase::writeConfig(offset, false))
    return false;

  uint16_t start = offset;

  if (! writeSchedulesConfig(offset)) {
    _log->println(F("Error writing schedules configuration!"));
    return false;
  }

  uint8_t crc = crc8EEPROM(start, offset);
  if (! writeEEPROM(offset, crc)) {
    _log->println(F("Error writing CRC!"));
    return false;
  }

  if (commit)
    commitConfig();

  if (! writeIRButtons()) {
    _log->println(F("Unable to write IR buttons configuration file!"));
  }

  return true;
}

void ESPIRBlaster::defaultConfig(uint8_t level) {
  if (level < 2) {
    ESPWebMQTTBase::defaultConfig(level);

    String str;

    if (level < 1) {
      str = FPSTR(overSSID);
      str += getBoardId();
      memset(_ssid, 0, sizeof(_ssid));
      strncpy(_ssid, str.c_str(), sizeof(_ssid) - 1);
    }
    str = FPSTR(overMQTTClient);
    str += getBoardId();
    memset(_mqttClient, 0, sizeof(_mqttClient));
    strncpy(_mqttClient, str.c_str(), sizeof(_mqttClient) - 1);
  }

  if (level < 3) {
    for (uint8_t i = 0; i < MAX_SCHEDULES; ++i) {
      schedules[i].clear();
      scheduleButtons[i] = -1;
    }

    clearIRButtons();
  }
}

void ESPIRBlaster::setupHttpServer() {
  ESPWebMQTTBase::setupHttpServer();

  httpServer->on(String(FPSTR(pathRemote)).c_str(), std::bind(&ESPIRBlaster::handleRemoteConfig, this));
  httpServer->on(String(FPSTR(pathGetRemote)).c_str(), std::bind(&ESPIRBlaster::handleGetRemote, this));
  httpServer->on(String(FPSTR(pathSetRemote)).c_str(), std::bind(&ESPIRBlaster::handleSetRemote, this));
#ifdef IRRX_PIN
  httpServer->on(String(FPSTR(pathRemoteData)).c_str(), std::bind(&ESPIRBlaster::handleRemoteData, this));
#endif
  httpServer->on(String(FPSTR(pathIRSend)).c_str(), std::bind(&ESPIRBlaster::handleIRSend, this));
  httpServer->on(String(FPSTR(pathSchedulesJs)).c_str(), std::bind(&ESPIRBlaster::handleSchedulesJs, this));
  httpServer->on(String(FPSTR(pathSchedules)).c_str(), std::bind(&ESPIRBlaster::handleSchedulesConfig, this));
  httpServer->on(String(FPSTR(pathGetSchedule)).c_str(), std::bind(&ESPIRBlaster::handleGetSchedule, this));
  httpServer->on(String(FPSTR(pathSetSchedule)).c_str(), std::bind(&ESPIRBlaster::handleSetSchedule, this));
}

void ESPIRBlaster::handleRootPage() {
  if (! userAuthenticate())
    return;

  String style = F("table {\n\
border-spacing: 2px;\n\
}\n\
td {\n\
width: 50px;\n\
height: 50px;\n\
border: 1px solid #ddd;\n\
border-radius: 4px;\n\
padding: 2px;\n\
text-align: center;\n\
position: relative;\n\
}\n\
.button {\n\
background-color: #ddd;\n\
border-color: black;\n\
cursor: pointer;\n\
}\n\
.number {\n\
font-size: x-small;\n\
position: absolute;\n\
bottom: 2px;\n\
right: 2px;\n\
}\n");

  String script = F("function sendIRButton(btn) {\n\
openUrl('");
  script += FPSTR(pathIRSend);
  script += F("?btn=' + btn + '&dummy=' + Date.now());\n\
}\n\
function uptimeToStr(uptime) {\n\
var tm, uptimestr = '';\n\
if (uptime >= 86400)\n\
uptimestr = parseInt(uptime / 86400) + ' day(s) ';\n\
tm = parseInt(uptime % 86400 / 3600);\n\
if (tm < 10)\n\
uptimestr += '0';\n\
uptimestr += tm + ':';\n\
tm = parseInt(uptime % 3600 / 60);\n\
if (tm < 10)\n\
uptimestr += '0';\n\
uptimestr += tm + ':';\n\
tm = parseInt(uptime % 60);\n\
if (tm < 10)\n\
uptimestr += '0';\n\
uptimestr += tm;\n\
return uptimestr;\n\
}\n\
function refreshData() {\n\
var request = getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathData);
  script += F("?dummy=' + Date.now(), true);\n\
request.onreadystatechange = function() {\n\
if (request.readyState == 4) {\n\
var data = JSON.parse(request.responseText);\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonMQTTConnected);
  script += F("').innerHTML = (data.");
  script += FPSTR(jsonMQTTConnected);
  script += F(" != true ? \"not \" : \"\") + \"connected\";\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonFreeHeap);
  script += F("').innerHTML = data.");
  script += FPSTR(jsonFreeHeap);
  script += F(";\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonUptime);
  script += F("').innerHTML = uptimeToStr(data.");
  script += FPSTR(jsonUptime);
  script += F(");\n");
  if (WiFi.getMode() == WIFI_STA) {
    script += FPSTR(getElementById);
    script += FPSTR(jsonRSSI);
    script += F("').innerHTML = data.");
    script += FPSTR(jsonRSSI);
    script += F(";\n");
  }
  script += F("}\n\
}\n\
request.send(null);\n\
}\n\
setInterval(refreshData, 500);\n");

  String page = ESPWebBase::webPageStart(F("IRblaster"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStyle(style);
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<h3>IRblaster</h3>\n\
<p>\n\
MQTT broker: <span id=\"");
  page += FPSTR(jsonMQTTConnected);
  page += F("\">?</span><br/>\n\
Heap free size: <span id=\"");
  page += FPSTR(jsonFreeHeap);
  page += F("\">0</span> bytes<br/>\n\
Uptime: <span id=\"");
  page += FPSTR(jsonUptime);
  page += F("\">?</span><br/>\n");
  if (WiFi.getMode() == WIFI_STA) {
    page += F("Signal strength: <span id=\"");
    page += FPSTR(jsonRSSI);
    page += F("\">?</span> dBm<br/>\n");
  }
  page += F("<p>\n\
<table cols=");
  page += String(BUTTON_COLS);
  page += F(">\n");
  for (uint8_t r = 0; r < BUTTON_ROWS; ++r) {
    page += F("<tr>");
    for (uint8_t c = 0; c < BUTTON_COLS; ++c) {
      if (irbuttons[r * BUTTON_COLS + c].rawBufLen) {
        page += F("<td class=\"button\" onclick=\"sendIRButton(");
        page += String(r * BUTTON_COLS + c);
        page += F(")\">");
      } else {
        page += F("<td>");
      }
      if (*irbuttons[r * BUTTON_COLS + c].buttonName)
        page += escapeQuote(irbuttons[r * BUTTON_COLS + c].buttonName);
      page += F("<span class=\"number\">");
      page += String(r * BUTTON_COLS + c + 1);
      page += F("</span></td>");
    }
    page += F("</tr>\n");
  }
  page += F("</table>\n\
<p>\n");
  page += navigator();
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPIRBlaster::handleRemoteConfig() {
  if (! adminAuthenticate())
    return;

  int8_t i;

  String style = F("table {\n\
border-collapse: collapse;\n\
}\n\
th, td {\n\
border-bottom: 1px solid #ddd;\n\
padding: 2px;\n\
}\n\
.modal {\n\
display: none;\n\
position: fixed;\n\
z-index: 1;\n\
left: 0;\n\
top: 0;\n\
width: 100%;\n\
height: 100%;\n\
overflow: auto;\n\
background-color: rgb(0,0,0);\n\
background-color: rgba(0,0,0,0.4);\n\
}\n\
.modal-content {\n\
background-color: #fefefe;\n\
margin: 15% auto;\n\
padding: 20px;\n\
border: 1px solid #888;\n\
width: 400px;\n\
}\n\
.close {\n\
color: #aaa;\n\
float: right;\n\
font-size: 28px;\n\
font-weight: bold;\n\
}\n\
.close:hover,\n\
.close:focus {\n\
color: black;\n\
text-decoration: none;\n\
cursor: pointer;\n\
}\n\
.hidden {\n\
display: none;\n\
}\n");

#ifdef IRRX_PIN
  String script = F("var currentInput=null;\n\
var timeoutId;\n\
function refreshData() {\n\
if (currentInput) {\n\
var request=getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathRemoteData);
  script += F("?dummy='+Date.now(), true);\n\
request.onreadystatechange=function() {\n\
if (request.readyState==4) {\n\
if (request.status==200) {\n\
var data=JSON.parse(request.responseText);\n\
currentInput.value=data.");
  script += FPSTR(jsonRemoteCode);
  script += F(";\n\
}\n\
timeoutId=setTimeout(refreshData, 500);\n\
}\n\
}\n\
request.send(null);\n\
}\n\
}\n\
function gotFocus(control) {\n\
currentInput=control;\n\
refreshData();\n\
}\n\
function lostFocus() {\n\
currentInput=null;\n\
clearTimeout(timeoutId);\n\
}\n");
#else
  String script = "";
#endif
  script += F("function loadData(form) {\n\
var request = getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathGetRemote);
  script += F("?id=' + form.id.value + '&dummy=' + Date.now(), false);\n\
request.send(null);\n\
if (request.status == 200) {\n\
var data = JSON.parse(request.responseText);\n\
form.");
  script += FPSTR(paramRemoteBtnName);
  script += F(".value = data.");
  script += FPSTR(paramRemoteBtnName);
  script += F(";\n\
form.");
  script += FPSTR(paramRemoteBtnCode);
  script += F(".value = data.");
  script += FPSTR(paramRemoteBtnCode);
  script += F(";\n\
form.");
  script += FPSTR(paramRemoteBtnRepeat);
  script += F(".value = data.");
  script += FPSTR(paramRemoteBtnRepeat);
  script += F(";\n\
form.");
  script += FPSTR(paramRemoteBtnGap);
  script += F(".value = data.");
  script += FPSTR(paramRemoteBtnGap);
  script += F(";\n\
}\n\
}\n\
function openForm(form, id) {\n\
form.id.value = id;\n\
loadData(form);\n\
document.getElementById(\"form\").style.display = \"block\";\n\
}\n\
function closeForm() {\n\
document.getElementById(\"form\").style.display = \"none\";\n\
}\n\
function trimString(field) {\n\
field.value = field.value.trim();\n\
}\n\
function fixNumber(field, minvalue, maxvalue) {\n\
var val = parseInt(field.value);\n\
if (isNaN(val) || (val < minvalue))\n\
field.value = minvalue;\n\
else\n\
if (val > maxvalue)\n\
field.value = maxvalue;\n\
}\n");

  String page = ESPWebBase::webPageStart(F("Remote Setup"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStyle(style);
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<table><caption><h3>Remote Setup</h3></caption>\n\
<tr><th>#</th><th>Button name</th><th>Repeat</th><th>Gap</th><th>Raw length</th></tr>\n");

  for (i = 0; i < BUTTON_COLS * BUTTON_ROWS; ++i) {
    page += F("<tr><td><a href=\"#\" onclick=\"openForm(document.form, ");
    page += String(i);
    page += F(")\">");
    page += String(i + 1);
    page += F("</a></td><td>");
    page += escapeQuote(irbuttons[i].buttonName);
    page += F("</td><td>");
    page += String(irbuttons[i].repeat + 1);
    page += F("</td><td>");
    page += String(irbuttons[i].gap);
    page += F("</td><td>");
    page += String(irbuttons[i].rawBufLen);
    page += F("</td></tr>\n");
  }
  page += F("</table>\n\
<p>\n\
<i>Don't forget to save changes!</i>\n\
<p>\n");

  page += ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Save"), String(F("onclick=\"location.href='")) + String(FPSTR(pathStore)) + String(F("?reboot=0'\"")));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::tagInput(FPSTR(typeHidden), FPSTR(paramReboot), "0");
  page += F("\n\
<div id=\"form\" class=\"modal\">\n\
<div class=\"modal-content\">\n\
<span class=\"close\" onclick=\"closeForm()\">&times;</span>\n\
<form name=\"form\" method=\"POST\" action=\"");
  page += FPSTR(pathSetRemote);
  page += F("\" onsubmit=\"closeForm()\">\n\
<input type=\"hidden\" name=\"id\" value=\"0\">\n\
<label>IR button name:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramRemoteBtnName), FPSTR(strEmpty), F("onblur=\"trimString(this)\""), BUTTON_NAME_SIZE - 1, BUTTON_NAME_SIZE - 1);
  page += F("<br/>\n\
<label>Repeat:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramRemoteBtnRepeat), F("1"), F("onblur=\"fixNumber(this, 1, 16)\""), 2, 2);
  page += F("<br/>\n\
<label>Gap (in ms):</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramRemoteBtnGap), F("0"), F("onblur=\"fixNumber(this, 0, 4095)\""), 4, 4);
  page += F("<br/>\n\
<label>Raw codes:</label><br/>\n\
<textarea cols=48 rows=4 name=\"");
  page += FPSTR(paramRemoteBtnCode);
#ifdef IRRX_PIN
  page += F("\" onfocus=\"gotFocus(this)\" onblur=\"lostFocus()");
#endif
  page += F("\"></textarea>\n\
<p>\n\
<input type=\"submit\" value=\"Update\">\n\
</form>\n\
</div>\n\
</div>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPIRBlaster::handleGetRemote() {
  int id = -1;

  if (httpServer->hasArg("id"))
    id = httpServer->arg("id").toInt();

  if ((id >= 0) && (id < BUTTON_COLS * BUTTON_ROWS)) {
    String page;

    page += charOpenBrace;
    page += charQuote;
    page += FPSTR(paramRemoteBtnName);
    page += F("\":\"");
    page += irbuttons[id].buttonName;
    page += F("\",\"");
    page += FPSTR(paramRemoteBtnCode);
    page += F("\":\"");
    for (uint16_t i = 0; i < irbuttons[id].rawBufLen; ++i) {
      if (i)
        page += charComma;
      page += String(irbuttons[id].rawBuf[i]);
    }
    page += F("\",\"");
    page += FPSTR(paramRemoteBtnRepeat);
    page += F("\":");
    page += String(irbuttons[id].repeat + 1);
    page += F(",\"");
    page += FPSTR(paramRemoteBtnGap);
    page += F("\":");
    page += String(irbuttons[id].gap);
    page += charCloseBrace;

    httpServer->send(200, FPSTR(textJson), page);
  } else {
    httpServer->send(204, FPSTR(textJson), strEmpty); // No content
  }
}

void ESPIRBlaster::handleSetRemote() {
  String argName, argValue;
  int8_t id = -1;
  irbutton_t irbutton;

  memset(&irbutton, 0, sizeof(irbutton_t));
  for (byte i = 0; i < httpServer->args(); i++) {
    argName = httpServer->argName(i);
    argValue = httpServer->arg(i);
    if (argName.equals("id")) {
      id = argValue.toInt();
    } else if (argName.equals(FPSTR(paramRemoteBtnName))) {
      strncpy(irbutton.buttonName, argValue.c_str(), sizeof(irbutton.buttonName) - 1);
    } else if (argName.equals(FPSTR(paramRemoteBtnCode))) {
      uint16_t j = 0, l = argValue.length();

      while ((j < l) && ((argValue[j] < '0') || (argValue[j] > '9'))) // Skip leading delimiter(s)
        ++j;
      while (j < l) {
        while ((j < l) && ((argValue[j] >= '0') && (argValue[j] <= '9'))) {
          irbutton.rawBuf[irbutton.rawBufLen] *= 10;
          irbutton.rawBuf[irbutton.rawBufLen] += (argValue[j] - '0');
          ++j;
        }
        if (++irbutton.rawBufLen >= IR_CAPTURE_BUFFER_SIZE)
          break;
        while ((j < l) && ((argValue[j] < '0') || (argValue[j] > '9'))) // Skip delimiter(s)
          ++j;
      }
    } else if (argName.equals(FPSTR(paramRemoteBtnRepeat))) {
      irbutton.repeat = constrain(argValue.toInt(), 1, 16) - 1;
    } else if (argName.equals(FPSTR(paramRemoteBtnGap))) {
      irbutton.gap = constrain(argValue.toInt(), 0, 4095);
    } else {
      _log->print(F("Unknown parameter \""));
      _log->print(argName);
      _log->print(F("\"!"));
    }
  }

  if ((id >= 0) && (id < BUTTON_COLS * BUTTON_ROWS)) {
    memcpy(&irbuttons[id], &irbutton, sizeof(irbutton_t));

    String page = ESPWebBase::webPageStart(F("Store IR Button"));
    page += F("<meta http-equiv=\"refresh\" content=\"1;URL=");
    page += FPSTR(pathRemote);
    page += F("\">\n");
    page += ESPWebBase::webPageStdStyle();
    page += ESPWebBase::webPageBody();
    page += F("Configuration stored successfully.\n\
Wait for 1 sec. to return to previous page.\n");
    page += ESPWebBase::webPageEnd();

    httpServer->send(200, FPSTR(textHtml), page);
  } else {
    httpServer->send(204, FPSTR(textHtml), strEmpty);
  }
}

#ifdef IRRX_PIN
void ESPIRBlaster::handleRemoteData() {
  static uint32_t lastTime = 0;

  if ((! rawBufLen) || (millis() - lastTime > 1000)) {
    httpServer->send(204, FPSTR(textJson), strEmpty); // No content
  } else {
    String page;

    page += charOpenBrace;
    page += charQuote;
    page += FPSTR(jsonRemoteCode);
    page += F("\":\"");
    for (uint16_t i = 0; i < rawBufLen; ++i) {
      if (i)
        page += charComma;
      page += String(rawBuf[i]);
    }
    page += charQuote;
    page += charCloseBrace;

    httpServer->send(200, FPSTR(textJson), page);
  }

  rawBufLen = 0;
  lastTime = millis();
}
#endif

void ESPIRBlaster::handleIRSend() {
  int8_t btn = -1;

  if (httpServer->hasArg(F("btn")))
    btn = httpServer->arg(F("btn")).toInt();
  if ((btn >= 0) && (btn < BUTTON_COLS * BUTTON_ROWS) && irbuttons[btn].rawBufLen) {
    sendButtonCode(btn);
  }

  httpServer->send(200, FPSTR(textPlain), strEmpty);
}

void ESPIRBlaster::handleSchedulesJs() {
  String script = F("function loadData(form) {\n\
var request = getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathGetSchedule);
  script += F("?id=' + form.id.value + '&dummy=' + Date.now(), false);\n\
request.send(null);\n\
if (request.status == 200) {\n\
var data = JSON.parse(request.responseText);\n\
form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value = data.");
  script += FPSTR(paramSchedulePeriod);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleHour);
  script += F(".value = data.");
  script += FPSTR(paramScheduleHour);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleMinute);
  script += F(".value = data.");
  script += FPSTR(paramScheduleMinute);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleSecond);
  script += F(".value = data.");
  script += FPSTR(paramScheduleSecond);
  script += F(";\n\
if (data.");
  script += FPSTR(paramSchedulePeriod);
  script += F(" == 3) {\n\
var weekdaysdiv = document.getElementById('weekdays');\n\
var elements = weekdaysdiv.getElementsByTagName('input');\n\
for (var i = 0; i < elements.length; i++) {\n\
if (elements[i].type == 'checkbox') {\n\
if ((data.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(" & elements[i].value) != 0)\n\
elements[i].checked = true;\n\
else\n\
elements[i].checked = false;\n\
}\n\
}\n\
form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value = data.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(";\n\
} else {\n\
form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value = 0;\n\
form.");
  script += FPSTR(paramScheduleDay);
  script += F(".value = data.");
  script += FPSTR(paramScheduleDay);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleMonth);
  script += F(".value = data.");
  script += FPSTR(paramScheduleMonth);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleYear);
  script += F(".value = data.");
  script += FPSTR(paramScheduleYear);
  script += F(";\n\
}\n\
form.");
  script += FPSTR(paramScheduleIRButton);
  script += F(".value = data.");
  script += FPSTR(paramScheduleIRButton);
  script += F(";\n\
}\n\
}\n\
function openForm(form, id) {\n\
form.id.value = id;\n\
loadData(form);\n\
form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".onchange();\n\
document.getElementById(\"form\").style.display = \"block\";\n\
}\n\
function closeForm() {\n\
document.getElementById(\"form\").style.display = \"none\";\n\
}\n\
function checkNumber(field, minvalue, maxvalue) {\n\
var val = parseInt(field.value);\n\
if (isNaN(val) || (val < minvalue) || (val > maxvalue))\n\
return false;\n\
return true;\n\
}\n\
function validateForm(form) {\n\
if (form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value > 0) {\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value > 2) && (! checkNumber(form.");
  script += FPSTR(paramScheduleHour);
  script += F(", 0, 23))) {\n\
alert(\"Wrong hour!\");\n\
form.");
  script += FPSTR(paramScheduleHour);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value > 1) && (! checkNumber(form.");
  script += FPSTR(paramScheduleMinute);
  script += F(", 0, 59))) {\n\
alert(\"Wrong minute!\");\n\
form.");
  script += FPSTR(paramScheduleMinute);
  script += F(".focus();\n\
return false;\n\
}\n\
if (! checkNumber(form.");
  script += FPSTR(paramScheduleSecond);
  script += F(", 0, 59)) {\n\
alert(\"Wrong second!\");\n\
form.");
  script += FPSTR(paramScheduleSecond);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value == 3) && (form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value == 0)) {\n\
alert(\"None of weekdays selected!\");\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value >= 4) && (! checkNumber(form.");
  script += FPSTR(paramScheduleDay);
  script += F(", 1, ");
  script += String(Schedule::LASTDAYOFMONTH);
  script += F("))) {\n\
alert(\"Wrong day!\");\n\
form.");
  script += FPSTR(paramScheduleDay);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value >= 5) && (! checkNumber(form.");
  script += FPSTR(paramScheduleMonth);
  script += F(", 1, 12))) {\n\
alert(\"Wrong month!\");\n\
form.");
  script += FPSTR(paramScheduleMonth);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value == 6) && (! checkNumber(form.");
  script += FPSTR(paramScheduleYear);
  script += F(", 2017, 2099))) {\n\
alert(\"Wrong year!\");\n\
form.");
  script += FPSTR(paramScheduleYear);
  script += F(".focus();\n\
return false;\n\
}\n\
}\n\
return true;\n\
}\n\
function periodChanged(period) {\n\
document.getElementById(\"time\").style.display = (period.value != 0) ? \"inline\" : \"none\";\n\
document.getElementById(\"hh\").style.display = (period.value > 2) ? \"inline\" : \"none\";\n\
document.getElementById(\"mm\").style.display = (period.value > 1) ? \"inline\" : \"none\";\n\
document.getElementById(\"weekdays\").style.display = (period.value == 3) ? \"block\" : \"none\";\n\
document.getElementById(\"date\").style.display = (period.value > 3) ? \"block\" : \"none\";\n\
document.getElementById(\"month\").style.display = (period.value > 4) ? \"inline\" : \"none\";\n\
document.getElementById(\"year\").style.display = (period.value == 6) ? \"inline\" : \"none\";\n\
document.getElementById(\"button\").style.display = (period.value != 0) ? \"block\" : \"none\";\n\
}\n\
function weekChanged(wd) {\n\
var weekdays = document.form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value;\n\
if (wd.checked == \"\") weekdays &= ~wd.value; else weekdays |= wd.value;\n\
document.form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value = weekdays;\n\
}\n\
function fixNumber(field, minvalue, maxvalue) {\n\
var val = parseInt(field.value);\n\
if (isNaN(val) || (val < minvalue))\n\
field.value = minvalue;\n\
else\n\
if (val > maxvalue)\n\
field.value = maxvalue;\n\
}\n");

  httpServer->send(200, FPSTR(applicationJavascript), script);
}

void ESPIRBlaster::handleSchedulesConfig() {
  if (! adminAuthenticate())
    return;

  int8_t i;

  String style = F("table {\n\
border-collapse: collapse;\n\
}\n\
th, td {\n\
border-bottom: 1px solid #ddd;\n\
padding: 2px;\n\
}\n\
.modal {\n\
display: none;\n\
position: fixed;\n\
z-index: 1;\n\
left: 0;\n\
top: 0;\n\
width: 100%;\n\
height: 100%;\n\
overflow: auto;\n\
background-color: rgb(0,0,0);\n\
background-color: rgba(0,0,0,0.4);\n\
}\n\
.modal-content {\n\
background-color: #fefefe;\n\
margin: 15% auto;\n\
padding: 20px;\n\
border: 1px solid #888;\n\
width: 400px;\n\
}\n\
.close {\n\
color: #aaa;\n\
float: right;\n\
font-size: 28px;\n\
font-weight: bold;\n\
}\n\
.close:hover,\n\
.close:focus {\n\
color: black;\n\
text-decoration: none;\n\
cursor: pointer;\n\
}\n\
.hidden {\n\
display: none;\n\
}\n");

  String page = ESPWebBase::webPageStart(F("Schedules Setup"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStyle(style);
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(FPSTR(pathSchedulesJs), true);
  page += ESPWebBase::webPageBody();
  page += F("<table><caption><h3>Schedules Setup</h3></caption>\n\
<tr><th>#</th><th>Event</th><th>Next time</th><th>IR button #</th></tr>\n");

  for (i = 0; i < MAX_SCHEDULES; ++i) {
    page += F("<tr><td><a href=\"#\" onclick=\"openForm(document.form, ");
    page += String(i);
    page += F(")\">");
    page += String(i + 1);
    page += F("</a></td><td>");
    page += schedules[i];
    page += F("</td><td>");
    page += schedules[i].nextTimeStr();
    page += F("</td><td>");
    if (schedules[i].period() != Schedule::NONE) {
      if (scheduleButtons[i] >= 0) {
        page += String(scheduleButtons[i] + 1);
        if (*irbuttons[scheduleButtons[i]].buttonName) {
          page += F(" (");
          page += escapeQuote(irbuttons[scheduleButtons[i]].buttonName);
          page += ')';
        }
      }
    }
    page += F("</td></tr>\n");
  }
  page += F("</table>\n\
<p>\n\
<i>Don't forget to save changes!</i>\n\
<p>\n");

  page += ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Save"), String(F("onclick=\"location.href='")) + String(FPSTR(pathStore)) + String(F("?reboot=0'\"")));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::tagInput(FPSTR(typeHidden), FPSTR(paramReboot), "0");
  page += F("\n\
<div id=\"form\" class=\"modal\">\n\
<div class=\"modal-content\">\n\
<span class=\"close\" onclick=\"closeForm()\">&times;</span>\n\
<form name=\"form\" method=\"GET\" action=\"");
  page += FPSTR(pathSetSchedule);
  page += F("\" onsubmit=\"if (validateForm(this)) closeForm(); else return false;\">\n\
<input type=\"hidden\" name=\"id\" value=\"0\">\n\
<select name=\"");
  page += FPSTR(paramSchedulePeriod);
  page += F("\" size=\"1\" onchange=\"periodChanged(this)\">\n\
<option value=\"0\">Never!</option>\n\
<option value=\"1\">Every minute</option>\n\
<option value=\"2\">Every hour</option>\n\
<option value=\"3\">Every week</option>\n\
<option value=\"4\">Every month</option>\n\
<option value=\"5\">Every year</option>\n\
<option value=\"6\">Once</option>\n\
</select>\n\
<span id=\"time\" class=\"hidden\">at\n\
<span id=\"hh\" class=\"hidden\">");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleHour), "0", F("onblur=\"fixNumber(this, 0, 23)\""), 2, 2);
  page += F("\n:</span>\n\
<span id=\"mm\" class=\"hidden\">");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleMinute), "0", F("onblur=\"fixNumber(this, 0, 59)\""), 2, 2);
  page += F("\n:</span>\n");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleSecond), "0", F("onblur=\"fixNumber(this, 0, 59)\""), 2, 2);
  page += F("</span><br/>\n\
<div id=\"weekdays\" class=\"hidden\">\n\
<input type=\"hidden\" name=\"");
  page += FPSTR(paramScheduleWeekdays);
  page += F("\" value=\"0\">\n");

  for (i = 0; i < 7; i++) {
    page += F("<input type=\"checkbox\" value=\"");
    page += String(1 << i);
    page += F("\" onchange=\"weekChanged(this)\">");
    page += weekdayName(i);
    page += charLF;
  }
  page += F("</div>\n\
<div id=\"date\" class=\"hidden\">\n\
<select name=\"");
  page += FPSTR(paramScheduleDay);
  page += F("\" size=\"1\">\n");

  for (i = 1; i <= 31; i++) {
    page += F("<option value=\"");
    page += String(i);
    page += F("\">");
    page += String(i);
    page += F("</option>\n");
  }
  page += F("<option value=\"");
  page += String(Schedule::LASTDAYOFMONTH);
  page += F("\">Last</option>\n\
</select>\n\
day\n\
<span id=\"month\" class=\"hidden\">of\n\
<select name=\"");
  page += FPSTR(paramScheduleMonth);
  page += F("\" size=\"1\">\n");

  for (i = 1; i <= 12; i++) {
    page += F("<option value=\"");
    page += String(i);
    page += F("\">");
    page += monthName(i);
    page += F("</option>\n");
  }
  page += F("</select>\n\
</span>\n\
<span id=\"year\" class=\"hidden\">");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleYear), "2017", F("onblur=\"fixNumber(this, 2017, 2099)\""), 4, 4);
  page += F("</span>\n\
</div>\n\
<div id=\"button\" class=\"hidden\">\n\
<label>Send IR button code</label><br/>\n\
<select name=\"");
  page += FPSTR(paramScheduleIRButton);
  page += F("\" size=\"1\">\n\
<option value=\"-1\">");
  page += FPSTR(strNone);
  page += F("</option>\n");

  for (i = 0; i < BUTTON_COLS * BUTTON_ROWS; ++i) {
    page += F("<option value=\"");
    page += String(i);
    if (! irbuttons[i].rawBufLen)
      page += F("\" disabled>");
    else
      page += F("\">");
    page += String(i + 1);
    if (*irbuttons[i].buttonName) {
      page += F(" (");
      page += escapeQuote(irbuttons[i].buttonName);
      page += ')';
    }
    page += F("</option>\n");
  }
  page += F("</select>\n\
</div>\n\
<p>\n\
<input type=\"submit\" value=\"Update\">\n\
</form>\n\
</div>\n\
</div>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPIRBlaster::handleGetSchedule() {
  int id = -1;

  if (httpServer->hasArg("id"))
    id = httpServer->arg("id").toInt();

  if ((id >= 0) && (id < MAX_SCHEDULES)) {
    String page;

    page += charOpenBrace;
    page += charQuote;
    page += FPSTR(paramSchedulePeriod);
    page += F("\":");
    page += String(schedules[id].period());
    page += F(",\"");
    page += FPSTR(paramScheduleHour);
    page += F("\":");
    page += String(schedules[id].hour());
    page += F(",\"");
    page += FPSTR(paramScheduleMinute);
    page += F("\":");
    page += String(schedules[id].minute());
    page += F(",\"");
    page += FPSTR(paramScheduleSecond);
    page += F("\":");
    page += String(schedules[id].second());
    page += F(",\"");
    page += FPSTR(paramScheduleWeekdays);
    page += F("\":");
    page += String(schedules[id].weekdays());
    page += F(",\"");
    page += FPSTR(paramScheduleDay);
    page += F("\":");
    page += String(schedules[id].day());
    page += F(",\"");
    page += FPSTR(paramScheduleMonth);
    page += F("\":");
    page += String(schedules[id].month());
    page += F(",\"");
    page += FPSTR(paramScheduleYear);
    page += F("\":");
    page += String(schedules[id].year());
    page += F(",\"");
    page += FPSTR(paramScheduleIRButton);
    page += F("\":");
    page += String(scheduleButtons[id]);
    page += charCloseBrace;

    httpServer->send(200, FPSTR(textJson), page);
  } else {
    httpServer->send(204, FPSTR(textJson), strEmpty); // No content
  }
}

void ESPIRBlaster::handleSetSchedule() {
  String argName, argValue;
  int8_t id = -1;
  Schedule::period_t period = Schedule::NONE;
  int8_t hour = -1;
  int8_t minute = -1;
  int8_t second = -1;
  uint8_t weekdays = 0;
  int8_t day = 0;
  int8_t month = 0;
  int16_t year = 0;
  int8_t button = -1;

  for (byte i = 0; i < httpServer->args(); i++) {
    argName = httpServer->argName(i);
    argValue = httpServer->arg(i);
    if (argName.equals("id")) {
      id = argValue.toInt();
    } else if (argName.equals(FPSTR(paramSchedulePeriod))) {
      period = (Schedule::period_t)argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleHour))) {
      hour = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleMinute))) {
      minute = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleSecond))) {
      second = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleWeekdays))) {
      weekdays = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleDay))) {
      day = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleMonth))) {
      month = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleYear))) {
      year = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleIRButton))) {
      button = constrain(argValue.toInt(), -1, BUTTON_COLS * BUTTON_ROWS - 1);
    } else {
      _log->print(F("Unknown parameter \""));
      _log->print(argName);
      _log->print(F("\"!"));
    }
  }

  if ((id >= 0) && (id < MAX_SCHEDULES)) {
    if (period == Schedule::NONE)
      schedules[id].clear();
    else
      schedules[id].set(period, hour, minute, second, weekdays, day, month, year);
    scheduleButtons[id] = button;

    String page = ESPWebBase::webPageStart(F("Store Schedule"));
    page += F("<meta http-equiv=\"refresh\" content=\"1;URL=");
    page += FPSTR(pathSchedules);
    page += F("\">\n");
    page += ESPWebBase::webPageStdStyle();
    page += ESPWebBase::webPageBody();
    page += F("Configuration stored successfully.\n\
Wait for 1 sec. to return to previous page.\n");
    page += ESPWebBase::webPageEnd();

    httpServer->send(200, FPSTR(textHtml), page);
  } else {
    httpServer->send(204, FPSTR(textHtml), strEmpty);
  }
}

String ESPIRBlaster::navigator() {
  String result = btnWiFiConfig();
  result += btnTimeConfig();
  result += btnMQTTConfig();
  result += btnRemoteConfig();
  result += btnSchedulesConfig();
  result += btnLog();
  result += btnReboot();

  return result;
}

String ESPIRBlaster::btnRemoteConfig() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Remote Setup"), String(F("onclick=\"location.href='")) + String(FPSTR(pathRemote)) + String(F("'\"")));
  result += charLF;

  return result;
}

String ESPIRBlaster::btnSchedulesConfig() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Schedules Setup"), String(F("onclick=\"location.href='")) + String(FPSTR(pathSchedules)) + String(F("'\"")));
  result += charLF;

  return result;
}

void ESPIRBlaster::mqttCallback(char *topic, byte *payload, unsigned int length) {
  ESPWebMQTTBase::mqttCallback(topic, payload, length);

  char* topicBody = topic + strlen(_mqttClient) + 1; // Skip "/ClientName" from topic
  if (! strcmp_P(topicBody, mqttRemoteBtnTopic)) {
    int8_t btn = -1;

    if (length)
      btn = atoi((char*)payload);
    if ((btn > 0) && (btn <= BUTTON_COLS * BUTTON_ROWS)) {
      sendButtonCode(btn - 1);
    } else
      _log->println(F("Wrong IR button index!"));
  } else {
    _log->println(F("Unexpected topic!"));
  }
}

void ESPIRBlaster::mqttResubscribe() {
  String topic;

  if (_mqttClient != strEmpty) {
    topic += charSlash;
    topic += _mqttClient;
  }
  topic += FPSTR(mqttRemoteBtnTopic);
  mqttSubscribe(topic);
}

bool ESPIRBlaster::readSchedulesConfig(uint16_t &offset) {
  Schedule::period_t period;
  int8_t hour;
  int8_t minute;
  int8_t second;
  uint8_t weekdays;
  int8_t day;
  int8_t month;
  int16_t year;

  for (int8_t i = 0; i < MAX_SCHEDULES; ++i) {
    if ((! readEEPROM(offset, (uint8_t*)&period, sizeof(period))) || (! readEEPROM(offset, (uint8_t*)&hour, sizeof(hour))) ||
      (! readEEPROM(offset, (uint8_t*)&minute, sizeof(minute))) || (! readEEPROM(offset, (uint8_t*)&second, sizeof(second))))
      return false;
    if (period == Schedule::WEEKLY) {
      if (! readEEPROM(offset, (uint8_t*)&weekdays, sizeof(weekdays)))
        return false;
    } else {
      if ((! readEEPROM(offset, (uint8_t*)&day, sizeof(day))) || (! readEEPROM(offset, (uint8_t*)&month, sizeof(month))) ||
        (! readEEPROM(offset, (uint8_t*)&year, sizeof(year))))
        return false;
    }
    if (! readEEPROM(offset, (uint8_t*)&scheduleButtons[i], sizeof(scheduleButtons[i])))
      return false;

    if (period == Schedule::NONE)
      schedules[i].clear();
    else
      schedules[i].set(period, hour, minute, second, weekdays, day, month, year);
  }

  return true;
}

bool ESPIRBlaster::writeSchedulesConfig(uint16_t &offset) {
  Schedule::period_t period;
  int8_t hour;
  int8_t minute;
  int8_t second;
  uint8_t weekdays;
  int8_t day;
  int8_t month;
  int16_t year;

  for (int8_t i = 0; i < MAX_SCHEDULES; ++i) {
    period = schedules[i].period();
    hour = schedules[i].hour();
    minute = schedules[i].minute();
    second = schedules[i].second();
    if (period == Schedule::WEEKLY) {
      weekdays = schedules[i].weekdays();
    } else {
      day = schedules[i].day();
      month = schedules[i].month();
      year = schedules[i].year();
    }

    if ((! writeEEPROM(offset, (uint8_t*)&period, sizeof(period))) || (! writeEEPROM(offset, (uint8_t*)&hour, sizeof(hour))) ||
      (! writeEEPROM(offset, (uint8_t*)&minute, sizeof(minute))) || (! writeEEPROM(offset, (uint8_t*)&second, sizeof(second))))
      return false;
    if (period == Schedule::WEEKLY) {
      if (! writeEEPROM(offset, (uint8_t*)&weekdays, sizeof(weekdays)))
        return false;
    } else {
      if ((! writeEEPROM(offset, (uint8_t*)&day, sizeof(day))) || (! writeEEPROM(offset, (uint8_t*)&month, sizeof(month))) ||
        (! writeEEPROM(offset, (uint8_t*)&year, sizeof(year))))
        return false;
    }
    if (! writeEEPROM(offset, (uint8_t*)&scheduleButtons[i], sizeof(scheduleButtons[i])))
      return false;
  }

  return true;
}

static const uint32_t IR_SIGNATURE = 0x42524923; // "#IRB"

static uint16_t memcrc16(const uint8_t *pcBlock, size_t len) {
  uint16_t crc = 0xFFFF;

  while (len--) {
    crc ^= *pcBlock++ << 8;

    for (uint8_t i = 0; i < 8; ++i)
      crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
  }

  return crc;
}

bool ESPIRBlaster::readIRButtons() {
  static const char strError[] PROGMEM = "Error reading from file!";

  File file;
  uint32_t sign;
  uint16_t crc;

  _log->println(F("Reading IR buttons configuration file"));
  file = SPIFFS.open(FPSTR(remoteFileName), "r");
  if (! file) {
    _log->println(F("Error opening file!"));
    return false;
  }
  if ((file.read((uint8_t*)&sign, sizeof(sign)) != sizeof(sign)) || (sign != IR_SIGNATURE)) {
    file.close();
    _log->println(F("Error reading or illegal signature!"));
    return false;
  }
  memset(irbuttons, 0, sizeof(irbuttons));
  for (uint8_t i = 0; i < BUTTON_COLS * BUTTON_ROWS; ++i) {
    if (file.read((uint8_t*)&irbuttons[i], sizeof(irbuttons[i]) - sizeof(irbuttons[i].rawBuf)) != sizeof(irbuttons[i]) - sizeof(irbuttons[i].rawBuf)) { // read without rawBuf field
      file.close();
      _log->println(FPSTR(strError));
      return false;
    }
    if (irbuttons[i].rawBufLen) {
      if (file.read((uint8_t*)irbuttons[i].rawBuf, sizeof(uint16_t) * irbuttons[i].rawBufLen) != sizeof(uint16_t) * irbuttons[i].rawBufLen) { // read rawBuf field
        file.close();
        _log->println(FPSTR(strError));
        return false;
      }
    }
  }
  if ((file.read((uint8_t*)&crc, sizeof(crc)) != sizeof(crc)) || (crc != memcrc16((uint8_t*)irbuttons, sizeof(irbuttons)))) {
    file.close();
    _log->println(F("Error reading or illegal CRC!"));
    return false;
  }
  file.close();

  return true;
}

bool ESPIRBlaster::writeIRButtons() {
  static const char strError[] PROGMEM = "Error writing to file!";

  File file;
  uint32_t sign = IR_SIGNATURE;
  uint16_t crc;

  _log->println(F("Writing IR buttons configuration file"));
  file = SPIFFS.open(FPSTR(remoteFileName), "w");
  if (! file) {
    _log->println(F("Error creating file!"));
    return false;
  }
  if (file.write((uint8_t*)&sign, sizeof(sign)) != sizeof(sign)) {
    file.close();
    _log->println(FPSTR(strError));
    return false;
  }
  for (uint8_t i = 0; i < BUTTON_COLS * BUTTON_ROWS; ++i) {
    if (file.write((uint8_t*)&irbuttons[i], sizeof(irbuttons[i]) - sizeof(irbuttons[i].rawBuf)) != sizeof(irbuttons[i]) - sizeof(irbuttons[i].rawBuf)) { // write without rawBuf field
      file.close();
      _log->println(FPSTR(strError));
      return false;
    }
    if (irbuttons[i].rawBufLen) {
      if (file.write((uint8_t*)irbuttons[i].rawBuf, sizeof(uint16_t) * irbuttons[i].rawBufLen) != sizeof(uint16_t) * irbuttons[i].rawBufLen) { // write rawBuf field
        file.close();
        _log->println(FPSTR(strError));
        return false;
      }
    }
  }
  crc = memcrc16((uint8_t*)irbuttons, sizeof(irbuttons));
  if (file.write((uint8_t*)&crc, sizeof(crc)) != sizeof(crc)) {
    file.close();
    _log->println(F("Error writing CRC to file!"));
    return false;
  }
  file.close();

  return true;
}

void ESPIRBlaster::clearIRButtons() {
  memset(irbuttons, 0, sizeof(irbuttons));
}

#ifdef IRRX_PIN
bool ESPIRBlaster::cloneRemoteCode(decode_results *results) {
  if (results->repeat) {
    _log->println(F("IR sequence is repeat code, ignored!"));
    return false;
  }
  if (results->overflow) {
    _log->println(F("IR sequence too big!"));
    return false;
  }

  memset(rawBuf, 0, sizeof(rawBuf));
  rawBufLen = 0;
  for (uint16_t i = 1; i < results->rawlen; ++i) {
    uint32_t usecs;

    for (usecs = results->rawbuf[i] * RAWTICK; usecs > UINT16_MAX; usecs -= UINT16_MAX) {
      rawBuf[rawBufLen++] = UINT16_MAX;
      rawBuf[rawBufLen++] = 0;
    }
    rawBuf[rawBufLen++] = usecs;
  }
  _log->print(F("IR raw code buffer length: "));
  _log->println(rawBufLen);

  return true;
}
#endif

void ESPIRBlaster::sendButtonCode(uint8_t btn) {
  if ((btn >= BUTTON_COLS * BUTTON_ROWS) || (! irbuttons[btn].rawBufLen)) // Wrong IR button index or empty code!
    return;
#ifdef IRRX_PIN
  irRX->disableIRIn();
#endif

  uint8_t repeat = irbuttons[btn].repeat + 1;
  while (repeat--) {
    irTX->sendRaw(irbuttons[btn].rawBuf, irbuttons[btn].rawBufLen, 38);
    if (repeat)
      delay(irbuttons[btn].gap);
  }

#ifdef IRRX_PIN
  irRX->enableIRIn();
#endif

  logDateTime();
  _log->print(F(" code for IR button #"));
  _log->print(btn + 1);
  if (*irbuttons[btn].buttonName) {
    _log->print(F(" ("));
    _log->print(irbuttons[btn].buttonName);
    _log->print(')');
  }
  _log->println(F(" sended"));
}

ESPIRBlaster *app = new ESPIRBlaster();

void setup() {
#ifndef NOSERIAL
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println();
#endif

  app->setup();
}

void loop() {
  app->loop();
}
