extern "C" {
#include <sntp.h>
}
#include <pgmspace.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include "ESPWeb.h"
#include <EEPROM.h>
#include "Date.h"
#include "RTCmem.h"

const uint16_t EEPROM_SIZE = 4096;

static void halt() {
#ifdef LED_PIN
  digitalWrite(LED_PIN, HIGH); // Гасим светодиод
#endif
#ifndef NOSERIAL
  Serial.println(F("System halted!"));
  Serial.flush();
#endif
  while (true) {
    delay(1);
  }
}

/*
 *   ESPWebBase class implementation
 */

ESPWebBase::ESPWebBase() {
  httpServer = new ESP8266WebServer(80);
#ifdef LED_PIN
  _ticker = new Ticker();
#endif
#ifndef NOSERIAL
  _log = new StringLog(&Serial);
#else
  _log = new StringLog();
#endif
}

void ESPWebBase::setup() {
#ifdef LED_PIN
  analogWriteRange(255);
  pinMode(LED_PIN, OUTPUT);
#endif

  EEPROM.begin(EEPROM_SIZE);

  if (! SPIFFS.begin()) {
    _log->println(F("Unable to mount SPIFFS!"));
  }

  uint16_t offset = 0;

  if (! readConfig(offset)) {
    _log->println(F("EEPROM is empty or corrupt!"));
  }

  offset = 0;
  if (! readRTCmemory(offset)) {
    _log->println(F("RTC memory is empty!"));
  }

  if (! WiFi.hostname(getHostName())) {
    _log->println(F("Unable to change host name!"));
  }

  if (*_ntpServer1 || *_ntpServer2 || *_ntpServer3) {
    sntp_set_timezone(_ntpTimeZone);
    if (*_ntpServer1)
      sntp_setservername(0, _ntpServer1);
    if (*_ntpServer2)
      sntp_setservername(1, _ntpServer2);
    if (*_ntpServer3)
      sntp_setservername(2, _ntpServer3);
    sntp_init();
  }
  _lastNtpTime = 0;
  _lastNtpUpdate = 0;

  setupExtra();

  setupWiFi();
  setupHttpServer();
}

void ESPWebBase::loop() {
  const uint32_t timeout = 300000; // 5 min.
  static uint32_t nextTime = timeout;

  if ((!_apMode) && (WiFi.status() != WL_CONNECTED) && ((WiFi.getMode() == WIFI_STA) || ((int32_t)(millis() - nextTime) >= 0))) {
    setupWiFi();
    nextTime = millis() + timeout;
  }

  httpServer->handleClient();
  loopExtra();

  delay(1); // For WiFi maintenance
}

void ESPWebBase::reboot() {
  cleanup();
#ifndef NOSERIAL
  Serial.println(F("Rebooting..."));
  Serial.flush();
#endif
  ESP.restart();
}

void ESPWebBase::cleanup() {
#ifdef LED_PIN
  disablePulse();
  digitalWrite(LED_PIN, HIGH);
#endif
}

void ESPWebBase::setupExtra() {
  // Stub
}

void ESPWebBase::loopExtra() {
  // Stub
}

String ESPWebBase::getBoardId() {
  String result;

  result = String(ESP.getChipId(), HEX);
  result.toUpperCase();

  return result;
}

String ESPWebBase::getHostName() {
  String result;

  result = FPSTR(defSSID);
  result += getBoardId();

  return result;
}

bool ESPWebBase::readRTCmemory(uint16_t &offset) {
  uint32_t sign;

//  _log->println(F("Reading RTC memory"));
  RTCmem.get(offset, sign);
  offset += sizeof(sign);
  if (sign != SIGNATURE) {
    _log->println(F("No signature found in RTC!"));
    return false;
  }

  return true;
}

bool ESPWebBase::writeRTCmemory(uint16_t &offset) {
  uint32_t sign = SIGNATURE;

//  _log->println(F("Writing config to RTC"));
  RTCmem.put(offset, sign);
  offset += sizeof(sign);

  return true;
}

uint8_t ESPWebBase::readEEPROM(uint16_t &offset) {
  return EEPROM.read(offset++);
}

bool ESPWebBase::readEEPROM(uint16_t &offset, uint8_t *buf, uint16_t len) {
  if (offset + len > EEPROM_SIZE)
    return false;

  while (len--)
    *buf++ = EEPROM.read(offset++);

  return true;
}

bool ESPWebBase::writeEEPROM(uint16_t &offset, uint8_t data) {
  if (offset >= EEPROM_SIZE)
    return false;

  EEPROM.write(offset++, data);

  return true;
}

bool ESPWebBase::writeEEPROM(uint16_t &offset, const uint8_t *buf, uint16_t len) {
  if (offset + len > EEPROM_SIZE)
    return false;

  while (len--)
    EEPROM.write(offset++, *buf++);

  return true;
}

bool ESPWebBase::readEEPROMString(uint16_t &offset, String &str, uint16_t maxlen) {
  if (offset + maxlen > EEPROM_SIZE)
    return false;

  str = strEmpty;
  while (maxlen--) {
    char c = readEEPROM(offset);
    if (! c)
      break;
    else
      str += c;
  }

  return true;
}

bool ESPWebBase::writeEEPROMString(uint16_t &offset, const String &str, uint16_t maxlen) {
  if (offset + maxlen > EEPROM_SIZE)
    return false;

  size_t slen = str.length();

  for (uint16_t i = 0; i < maxlen; ++i) {
    if (i < slen)
      writeEEPROM(offset, str[i]);
    else
      writeEEPROM(offset, 0);
  }

  return true;
}

void ESPWebBase::commitEEPROM() {
  EEPROM.commit();
}

void ESPWebBase::clearEEPROM() {
  uint16_t offset = 0;
  uint16_t size = EEPROM_SIZE;

  while (size--)
    writeEEPROM(offset, 0xFF);
  commitEEPROM();
  _log->println(F("EEPROM erased succefully!"));
}

uint8_t ESPWebBase::crc8EEPROM(uint16_t start, uint16_t end) {
  uint8_t crc = 0;

  while (start < end) {
    crc ^= readEEPROM(start);

    for (uint8_t i = 0; i < 8; ++i)
      crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
  }

  return crc;
}

bool ESPWebBase::readConfig(uint16_t &offset) {
  uint32_t sign;

  _log->println(F("Reading config from EEPROM"));
  if ((! readEEPROM(offset, (uint8_t*)&sign, sizeof(sign))) || (sign != SIGNATURE)) {
    _log->println(F("No signature found!"));
    defaultConfig();
    return false;
  }
  if ((! readEEPROM(offset, (uint8_t*)&_apMode, sizeof(_apMode))) || (! readEEPROM(offset, (uint8_t*)_ssid, sizeof(_ssid))) ||
    (! readEEPROM(offset, (uint8_t*)_password, sizeof(_password))) || (! readEEPROM(offset, (uint8_t*)_domain, sizeof(_domain))) ||
    (! readEEPROM(offset, (uint8_t*)_userName, sizeof(_userName))) || (! readEEPROM(offset, (uint8_t*)_userPassword, sizeof(_userPassword))) ||
    (! readEEPROM(offset, (uint8_t*)_adminName, sizeof(_adminName))) || (! readEEPROM(offset, (uint8_t*)_adminPassword, sizeof(_adminPassword))) ||
    (! readEEPROM(offset, (uint8_t*)_ntpServer1, sizeof(_ntpServer1))) || (! readEEPROM(offset, (uint8_t*)_ntpServer2, sizeof(_ntpServer2))) ||
    (! readEEPROM(offset, (uint8_t*)_ntpServer3, sizeof(_ntpServer3))) || (! readEEPROM(offset, (uint8_t*)&_ntpTimeZone, sizeof(_ntpTimeZone))) ||
    (! readEEPROM(offset, (uint8_t*)&_ntpUpdateInterval, sizeof(_ntpUpdateInterval)))) {
    _log->println(F("Error reading from EEPROM!"));
    defaultConfig();
    return false;
  }

  uint8_t crc = crc8EEPROM(0, offset);
  if (readEEPROM(offset) != crc) {
    _log->println(F("CRC mismatch! Use default WiFi parameters."));
    defaultConfig();
    return false;
  }

  return true;
}

bool ESPWebBase::writeConfig(uint16_t &offset, bool commit) {
  uint32_t sign = SIGNATURE;

  _log->println(F("Writing config to EEPROM"));
  if ((! writeEEPROM(offset, (uint8_t*)&sign, sizeof(sign))) || (! writeEEPROM(offset, (uint8_t*)&_apMode, sizeof(_apMode))) ||
    (! writeEEPROM(offset, (uint8_t*)_ssid, sizeof(_ssid))) || (! writeEEPROM(offset, (uint8_t*)_password, sizeof(_password))) ||
    (! writeEEPROM(offset, (uint8_t*)_domain, sizeof(_domain))) || (! writeEEPROM(offset, (uint8_t*)_userName, sizeof(_userName))) ||
    (! writeEEPROM(offset, (uint8_t*)_userPassword, sizeof(_userPassword))) || (! writeEEPROM(offset, (uint8_t*)_adminName, sizeof(_adminName))) ||
    (! writeEEPROM(offset, (uint8_t*)_adminPassword, sizeof(_adminPassword))) || (! writeEEPROM(offset, (uint8_t*)_ntpServer1, sizeof(_ntpServer1))) ||
    (! writeEEPROM(offset, (uint8_t*)_ntpServer2, sizeof(_ntpServer2))) || (! writeEEPROM(offset, (uint8_t*)_ntpServer3, sizeof(_ntpServer3))) ||
    (! writeEEPROM(offset, (uint8_t*)&_ntpTimeZone, sizeof(_ntpTimeZone))) || (! writeEEPROM(offset, (uint8_t*)&_ntpUpdateInterval, sizeof(_ntpUpdateInterval)))) {
    _log->println(F("Error writing to EEPROM!"));
    return false;
  }

  uint8_t crc = crc8EEPROM(0, offset);
  if (! writeEEPROM(offset, crc)) {
    _log->println(F("Error writing CRC!"));
    return false;
  }
  if (commit)
    commitConfig();

  return true;
}

inline void ESPWebBase::commitConfig() {
  commitEEPROM();
}

void ESPWebBase::defaultConfig(uint8_t level) {
  if (level < 1) {
    _apMode = true;
    String ssid = FPSTR(defSSID);
    ssid += getBoardId();
    memset(_ssid, 0, sizeof(_ssid));
    strncpy(_ssid, ssid.c_str(), sizeof(_ssid) - 1);
    memset(_password, 0, sizeof(_password));
    strncpy_P(_password, defPassword, sizeof(_password) - 1);
    memset(_domain, 0, sizeof(_domain));
    memset(_userName, 0, sizeof(_userName));
    strncpy_P(_userName, defUserName, sizeof(_userName) - 1);
    memset(_userPassword, 0, sizeof(_userPassword));
    memset(_adminName, 0, sizeof(_adminName));
    strncpy_P(_adminName, defAdminName, sizeof(_adminName) - 1);
    memset(_adminPassword, 0, sizeof(_adminPassword));
    strncpy_P(_adminPassword, defAdminPassword, sizeof(_adminPassword) - 1);
    memset(_ntpServer1, 0, sizeof(_ntpServer1));
    strncpy_P(_ntpServer1, defNtpServer, sizeof(_ntpServer1) - 1);
    memset(_ntpServer2, 0, sizeof(_ntpServer2));
    memset(_ntpServer3, 0, sizeof(_ntpServer3));
    _ntpTimeZone = defNtpTimeZone;
    _ntpUpdateInterval = defNtpUpdateInterval;
  }
}

bool ESPWebBase::setConfigParam(const String &name, const String &value) {
  if (name.equals(FPSTR(paramApMode)))
    _apMode = constrain(value.toInt(), 0, 1);
  else if (name.equals(FPSTR(paramSSID))) {
    memset(_ssid, 0, sizeof(_ssid));
    strncpy(_ssid, value.c_str(), sizeof(_ssid) - 1);
  } else if (name.equals(FPSTR(paramPassword))) {
    memset(_password, 0, sizeof(_password));
    strncpy(_password, value.c_str(), sizeof(_password) - 1);
  } else if (name.equals(FPSTR(paramDomain))) {
    memset(_domain, 0, sizeof(_domain));
    strncpy(_domain, value.c_str(), sizeof(_domain) - 1);
  } else if (name.equals(FPSTR(paramUserName))) {
    memset(_userName, 0, sizeof(_userName));
    strncpy(_userName, value.c_str(), sizeof(_userName) - 1);
  } else if (name.equals(FPSTR(paramUserPassword))) {
    memset(_userPassword, 0, sizeof(_userPassword));
    strncpy(_userPassword, value.c_str(), sizeof(_userPassword) - 1);
  } else if (name.equals(FPSTR(paramAdminName))) {
    memset(_adminName, 0, sizeof(_adminName));
    strncpy(_adminName, value.c_str(), sizeof(_adminName) - 1);
  } else if (name.equals(FPSTR(paramAdminPassword))) {
    memset(_adminPassword, 0, sizeof(_adminPassword));
    strncpy(_adminPassword, value.c_str(), sizeof(_adminPassword) - 1);
  } else if (name.equals(FPSTR(paramNtpServer1))) {
    memset(_ntpServer1, 0, sizeof(_ntpServer1));
    strncpy(_ntpServer1, value.c_str(), sizeof(_ntpServer1) - 1);
  } else if (name.equals(FPSTR(paramNtpServer2))) {
    memset(_ntpServer2, 0, sizeof(_ntpServer2));
    strncpy(_ntpServer2, value.c_str(), sizeof(_ntpServer2) - 1);
  } else if (name.equals(FPSTR(paramNtpServer3))) {
    memset(_ntpServer3, 0, sizeof(_ntpServer3));
    strncpy(_ntpServer3, value.c_str(), sizeof(_ntpServer3) - 1);
  } else if (name.equals(FPSTR(paramNtpTimeZone)))
    _ntpTimeZone = constrain(value.toInt(), -11, 13);
  else if (name.equals(FPSTR(paramNtpUpdateInterval)))
    _ntpUpdateInterval = _max(0, value.toInt()) * 1000;
  else
    return false;

  return true;
}

bool ESPWebBase::setupWiFiAsStation() {
  const uint32_t timeout = 60000; // 1 min.

  uint32_t maxTime = millis() + timeout;

  if (! *_ssid) {
    _log->println(F("Empty SSID!"));
    return false;
  }

  _log->print(F("Connecting to \""));
  _log->print(_ssid);
  _log->print(charQuote);

#ifdef LED_PIN
  enablePulse(PULSE);
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  while (WiFi.status() != WL_CONNECTED) {
    _log->print(charDot);
    delay(500);
    if ((int32_t)(millis() - maxTime) >= 0) {
      _log->println(F("FAIL!"));
#ifdef LED_PIN
      disablePulse();
#endif
      return false;
    }
  }
  _log->println(WiFi.localIP());
#ifdef LED_PIN
  enablePulse(BREATH);
#endif

  return true;
}

void ESPWebBase::setupWiFiAsAP() {
  String ssid, password;

  if (_apMode) {
    ssid = _ssid;
    password = _password;
  } else {
    ssid = getHostName();
    password = FPSTR(defPassword);
  }

  WiFi.mode(WIFI_AP);
//  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid.c_str(), password.c_str());

  _log->print(F("Configuring access point \""));
  _log->print(ssid);
  _log->print(F("\" with password \""));
  _log->print(password);
  _log->print(F("\" on IP address "));
  _log->println(WiFi.softAPIP());

#ifdef LED_PIN
  enablePulse(FADEIN);
#endif
}

void ESPWebBase::setupWiFi() {
  if (_apMode || (! setupWiFiAsStation()))
    setupWiFiAsAP();

  if (*_domain) {
    if (MDNS.begin(_domain)) {
      MDNS.addService("http", "tcp", 80);
      _log->println(F("mDNS responder started"));
    } else {
      _log->println(F("Error setting up mDNS responder!"));
    }
  }

  onWiFiConnected();
}

void ESPWebBase::onWiFiConnected() {
  httpServer->begin();
  _log->println(F("HTTP server started"));
}

bool ESPWebBase::userAuthenticate() {
  if (*_userName && *_userPassword) {
    if ((! httpServer->authenticate(_userName, _userPassword)) && (! httpServer->authenticate(_adminName, _adminPassword))) {
      httpServer->requestAuthentication();
      return false;
    }
  }
  return true;
}

bool ESPWebBase::adminAuthenticate() {
  if (*_adminName && *_adminPassword) {
    if (! httpServer->authenticate(_adminName, _adminPassword)) {
      httpServer->requestAuthentication();
      return false;
    }
  }
  return true;
}

uint32_t ESPWebBase::getTime() {
  if ((WiFi.getMode() == WIFI_STA) && (*_ntpServer1 || *_ntpServer2 || *_ntpServer3) && ((! _lastNtpTime) || (_ntpUpdateInterval && (millis() - _lastNtpUpdate >= _ntpUpdateInterval)))) {
    uint32_t now = sntp_get_current_timestamp();
    if (now > 1483228800UL) { // 01.01.2017 0:00:00
      _lastNtpTime = now;
      _lastNtpUpdate = millis();
      logDateTime(now);
      _log->println(F(" time updated successfully"));
    } else {
      const int32_t errorTimeout = 5000;
      static uint32_t lastError;

      if (millis() - lastError > errorTimeout) {
        _log->println(F("Unable to update time from NTP!"));
        lastError = millis();
      }
    }
  }
  if (_lastNtpTime)
    return _lastNtpTime + (millis() - _lastNtpUpdate) / 1000;
  else
    return 0;
}

void ESPWebBase::setTime(uint32_t now) {
  _lastNtpTime = now;
  _lastNtpUpdate = millis();
  logDateTime(now);
  _log->println(F(" time updated manualy"));
}

void ESPWebBase::logDate(uint32_t now) {
  if (! now)
    now = getTime();
  if (now)
    _log->print(dateToStr(now));
}

void ESPWebBase::logTime(uint32_t now) {
  if (! now)
    now = getTime();
  if (now)
    _log->print(timeToStr(now));
}

void ESPWebBase::logDateTime(uint32_t now) {
  if (! now)
    now = getTime();
  if (now)
    _log->print(dateTimeToStr(now));
}

void ESPWebBase::logTimeDate(uint32_t now) {
  if (! now)
    now = getTime();
  if (now)
    _log->print(timeDateToStr(now));
}

void ESPWebBase::setupHttpServer() {
  httpServer->onNotFound(std::bind(&ESPWebBase::handleNotFound, this));
  httpServer->on(String(FPSTR(pathStdCss)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleStdCss, this));
  httpServer->on(String(FPSTR(pathStdJs)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleStdJs, this));
  httpServer->on(strSlash, HTTP_GET, std::bind(&ESPWebBase::handleRootPage, this));
  httpServer->on(String(String(charSlash) + FPSTR(indexHtml)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleRootPage, this));
  httpServer->on(String(FPSTR(pathSPIFFS)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleSPIFFS, this));
  httpServer->on(String(FPSTR(pathSPIFFS)).c_str(), HTTP_POST, std::bind(&ESPWebBase::handleFileUploaded, this), std::bind(&ESPWebBase::handleFileUpload, this));
  httpServer->on(String(FPSTR(pathSPIFFS)).c_str(), HTTP_DELETE, std::bind(&ESPWebBase::handleFileDelete, this));
  httpServer->on(String(FPSTR(pathUpdate)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleUpdate, this));
  httpServer->on(String(FPSTR(pathUpdate)).c_str(), HTTP_POST, std::bind(&ESPWebBase::handleSketchUpdated, this), std::bind(&ESPWebBase::handleSketchUpdate, this));
  httpServer->on(String(FPSTR(pathWiFi)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleWiFiConfig, this));
  httpServer->on(String(FPSTR(pathTime)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleTimeConfig, this));
  httpServer->on(String(FPSTR(pathStore)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleStoreConfig, this));
  httpServer->on(String(FPSTR(pathStore)).c_str(), HTTP_POST, std::bind(&ESPWebBase::handleStoreConfig, this));
  httpServer->on(String(FPSTR(pathLog)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleLog, this));
  httpServer->on(String(FPSTR(pathClearLog)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleClearLog, this));
  httpServer->on(String(FPSTR(pathGetTime)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleGetTime, this));
  httpServer->on(String(FPSTR(pathSetTime)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleSetTime, this));
  httpServer->on(String(FPSTR(pathReboot)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleReboot, this));
  httpServer->on(String(FPSTR(pathData)).c_str(), HTTP_GET, std::bind(&ESPWebBase::handleData, this));
}

void ESPWebBase::handleStdCss() {
  String style = F("body {\n\
background-color: #eee;\n\
}");

  httpServer->send(200, FPSTR(textCss), style);
}

void ESPWebBase::handleStdJs() {
  String script = F("function getXmlHttpRequest() {\n\
var xmlhttp;\n\
try {\n\
xmlhttp=new ActiveXObject(\"Msxml2.XMLHTTP\");\n\
} catch(e) {\n\
try {\n\
xmlhttp=new ActiveXObject(\"Microsoft.XMLHTTP\");\n\
} catch(E) {\n\
xmlhttp=false;\n\
}\n\
}\n\
if ((!xmlhttp)&&(typeof XMLHttpRequest!='undefined')) {\n\
xmlhttp=new XMLHttpRequest();\n\
}\n\
return xmlhttp;\n\
}\n\
function openUrl(url) {\n\
var request=getXmlHttpRequest();\n\
request.open(\"GET\",url,false);\n\
request.send(null);\n\
}");

  httpServer->send(200, FPSTR(applicationJavascript), script);
}

void ESPWebBase::handleNotFound() {
  if (! handleFileRead(httpServer->uri()))
    httpServer->send(404, FPSTR(textPlain), FPSTR(fileNotFound));
}

void ESPWebBase::handleRootPage() {
  if (! userAuthenticate())
    return;

  String script = F("function refreshData() {\n\
var request = getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathData);
  script += F("?dummy=' + Date.now(), true);\n\
request.onreadystatechange = function() {\n\
if (request.readyState == 4) {\n\
var data = JSON.parse(request.responseText);\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonFreeHeap);
  script += F("').innerHTML = data.");
  script += FPSTR(jsonFreeHeap);
  script += F(";\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonUptime);
  script += F("').innerHTML = data.");
  script += FPSTR(jsonUptime);
  script += F(";\n");
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
setInterval(refreshData, 1000);\n");

  String page = ESPWebBase::webPageStart(F("ESP8266"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<h3>ESP8266</h3>\n\
<p>\n\
Heap free size: <span id=\"");
  page += FPSTR(jsonFreeHeap);
  page += F("\">0</span> bytes<br/>\n\
Uptime: <span id=\"");
  page += FPSTR(jsonUptime);
  page += F("\">0</span> seconds<br/>\n");
  if (WiFi.getMode() == WIFI_STA) {
    page += F("Signal strength: <span id=\"");
    page += FPSTR(jsonRSSI);
    page += F("\">?</span> dBm<br/>\n");
  }
  page += F("<p>\n");
  page += navigator();
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebBase::handleFileUploaded() {
  httpServer->send(200, FPSTR(textHtml), F("<META http-equiv=\"refresh\" content=\"2;URL=\">Upload successful."));
}

void ESPWebBase::handleFileUpload() {
  static File uploadFile;

  if (httpServer->uri() != FPSTR(pathSPIFFS))
    return;
  HTTPUpload& upload = httpServer->upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (! filename.startsWith(strSlash))
      filename = charSlash + filename;
    uploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile)
      uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile)
      uploadFile.close();
  }
}

void ESPWebBase::handleFileDelete() {
  if (httpServer->args() == 0)
    return httpServer->send(500, FPSTR(textPlain), F("BAD ARGS"));
  String path = httpServer->arg(0);
  if (path == strSlash)
    return httpServer->send(500, FPSTR(textPlain), F("BAD PATH"));
  if (! SPIFFS.exists(path))
    return httpServer->send(404, FPSTR(textPlain), FPSTR(fileNotFound));
  SPIFFS.remove(path);
  httpServer->send(200, FPSTR(textPlain), strEmpty);
  path = String();
}

void ESPWebBase::handleSPIFFS() {
  if (! adminAuthenticate())
    return;

  String script = FPSTR(getXmlHttpRequest);
  script += F("function openUrl(url, method) {\n\
var request = getXmlHttpRequest();\n\
request.open(method, url, false);\n\
request.send(null);\n\
if (request.status != 200)\n\
alert(request.responseText);\n\
}\n\
function getSelectedCount() {\n\
var inputs = document.getElementsByTagName(\"input\");\n\
var result = 0;\n\
for (var i = 0; i < inputs.length; i++) {\n\
if (inputs[i].type == \"checkbox\") {\n\
if (inputs[i].checked == true)\n\
result++;\n\
}\n\
}\n\
return result;\n\
}\n\
function updateSelected() {\n\
document.getElementsByName(\"delete\")[0].disabled = (getSelectedCount() > 0) ? false : true;\n\
}\n\
function deleteSelected() {\n\
var inputs = document.getElementsByTagName(\"input\");\n\
for (var i = 0; i < inputs.length; i++) {\n\
if (inputs[i].type == \"checkbox\") {\n\
if (inputs[i].checked == true)\n\
openUrl(\"");
  script += FPSTR(pathSPIFFS);
  script += F("?filename=/\" + encodeURIComponent(inputs[i].value) + '&dummy=' + Date.now(), \"DELETE\");\n\
}\n\
}\n\
location.reload(true);\n\
}\n");

  String page = ESPWebBase::webPageStart(F("SPIFFS"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<form method=\"POST\" action=\"\" enctype=\"multipart/form-data\" onsubmit=\"if (document.getElementsByName('upload')[0].files.length == 0) { alert('No file to upload!'); return false; }\">\n\
<h3>SPIFFS</h3>\n\
<p>\n");

  Dir dir = SPIFFS.openDir("/");
  int cnt = 0;
  while (dir.next()) {
    cnt++;
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    if (fileName.startsWith(strSlash))
      fileName = fileName.substring(1);
    page += F("<input type=\"checkbox\" name=\"file");
    page += String(cnt);
    page += F("\" value=\"");
    page += fileName;
    page += F("\" onchange=\"updateSelected()\"><a href=\"/");
    page += fileName;
    page += F("\" download>");
    page += fileName;
    page += F("</a>\t");
    page += String(fileSize);
    page += F("<br/>\n");
  }
  page += String(cnt);
  page += F(" file(s)<br/>\n\
<p>\n");
  page += ESPWebBase::tagInput(FPSTR(typeButton), F("delete"), F("Delete"), F("onclick=\"if (confirm('Are you sure to delete selected file(s)?') == true) deleteSelected()\" disabled"));
  page += F("\n\
<p>\n\
Upload new file:<br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeFile), F("upload"), strEmpty);
  page += charLF;
  page += ESPWebBase::tagInput(FPSTR(typeSubmit), strEmpty, F("Upload"));
  page += F("\n\
</form>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebBase::handleUpdate() {
  if (! adminAuthenticate())
    return;

  String page = ESPWebBase::webPageStart(F("Sketch Update"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageBody();
  page += F("<form method=\"POST\" action=\"\" enctype=\"multipart/form-data\" onsubmit=\"if (document.getElementsByName('update')[0].files.length == 0) { alert('No file to update!'); return false; }\">\n\
Select compiled sketch to upload:<br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeFile), F("update"), strEmpty);
  page += charLF;
  page += ESPWebBase::tagInput(FPSTR(typeSubmit), strEmpty, F("Update"));
  page += F("\n\
</form>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebBase::handleSketchUpdated() {
  static const char updateFailed[] PROGMEM = "Update failed!";
  static const char updateSuccess[] PROGMEM = "<META http-equiv=\"refresh\" content=\"15;URL=\">Update successful! Rebooting...";

  httpServer->send(200, FPSTR(textHtml), Update.hasError() ? FPSTR(updateFailed) : FPSTR(updateSuccess));

  ESP.restart();
}

void ESPWebBase::handleSketchUpdate() {
  if (httpServer->uri() != FPSTR(pathUpdate))
    return;
  HTTPUpload& upload = httpServer->upload();
  if (upload.status == UPLOAD_FILE_START) {
    cleanup();
    WiFiUDP::stopAll();
    _log->print(F("Update sketch from file \""));
    _log->print(upload.filename.c_str());
    _log->println(charQuote);
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (! Update.begin(maxSketchSpace)) { // start with max available size
#ifndef NOSERIAL
      Update.printError(Serial);
#endif
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
#ifndef NOSERIAL
    Serial.print(charDot);
#endif
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
#ifndef NOSERIAL
      Update.printError(Serial);
#endif
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { // true to set the size to the current progress
#ifndef NOSERIAL
      Serial.println();
#endif
      _log->print(F("Updated "));
      _log->print(upload.totalSize);
      _log->println(F(" byte(s) successful. Rebooting..."));
    } else {
#ifndef NOSERIAL
      Update.printError(Serial);
#endif
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
#ifndef NOSERIAL
      Serial.println();
#endif
    _log->println(F("Update was aborted"));
  }
  yield();
}

void ESPWebBase::handleWiFiConfig() {
  if (! adminAuthenticate())
    return;

  String script = F("function trimString(field) {\n\
field.value = field.value.trim();\n\
}\n\
function validateForm(form) {\n\
if (form.");
  script += FPSTR(paramSSID);
  script += F(".value == \"\") {\n\
alert(\"SSID must be set!\");\n\
form.");
  script += FPSTR(paramSSID);
  script += F(".focus();\n\
return false;\n\
}\n\
if (form.");
  script += FPSTR(paramPassword);
  script += F(".value == \"\") {\n\
alert(\"Password must be set!\");\n\
form.");
  script += FPSTR(paramPassword);
  script += F(".focus();\n\
return false;\n\
}\n\
form.scan.disabled = true;\n\
return true;\n\
}\n");

  String page = ESPWebBase::webPageStart(F("WiFi Setup"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<form name=\"wifi\" method=\"GET\" action=\"");
  page += FPSTR(pathStore);
  page += F("\" onsubmit=\"return validateForm(this)\">\n\
<h3>WiFi Setup</h3>\n\
<label>Mode:</label><br/>\n");
  if (_apMode)
    page += ESPWebBase::tagInput(FPSTR(typeRadio), FPSTR(paramApMode), "1", F("onclick=\"document.getElementById('scan').style.display='none'\" checked"));
  else
    page += ESPWebBase::tagInput(FPSTR(typeRadio), FPSTR(paramApMode), "1", F("onclick=\"document.getElementById('scan').style.display='none'\""));
  page += F("AP\n");
  if (! _apMode)
    page += ESPWebBase::tagInput(FPSTR(typeRadio), FPSTR(paramApMode), "0", F("onclick=\"document.getElementById('scan').style.display='block'\" checked"));
  else
    page += ESPWebBase::tagInput(FPSTR(typeRadio), FPSTR(paramApMode), "0", F("onclick=\"document.getElementById('scan').style.display='block'\""));
  page += F("Infrastructure\n<br/>\n\
<div id=\"scan\"");
  if (_apMode)
    page += F(" style=\"display:none\"");
  page += F(">\n\
<label>Available WiFi:</label><br/>\n\
<select name=\"scan\" size=5 onchange=\"wifi.");
  page += FPSTR(paramSSID);
  page += F(".value=this.value\">\n");

  int8_t n = WiFi.scanNetworks();
  if (n > 0) {
    for (int8_t i = 0; i < n; ++i) {
      page += F("<option value=\"");
      page += WiFi.SSID(i);
      page += F("\">");
      page += WiFi.SSID(i);
      page += F(" (");
      page += WiFi.RSSI(i);
      page += F(" dBm)</option>\n");
    }
  }
  page += F("</select>\n\
</div>\n\
<label>SSID:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramSSID), _ssid, F("onblur=\"trimString(this)\""), MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("\n*<br/>\n\
<label>Password:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typePassword), FPSTR(paramPassword), _password, MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("\n*<br/>\n\
<label>mDNS domain:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramDomain), _domain, F("onblur=\"trimString(this)\""), MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("\n\
.local (leave blank to ignore mDNS)\n\
<p>\n\
<label>User name:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramUserName), _userName, F("onblur=\"trimString(this)\""), MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("<br/>\n\
<label>User password:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typePassword), FPSTR(paramUserPassword), _userPassword, MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("<br/>\n\
<label>Admin name:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramAdminName), _adminName, F("onblur=\"trimString(this)\""), MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("<br/>\n\
<label>Admin password:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typePassword), FPSTR(paramAdminPassword), _adminPassword, MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("\n\
<p>\n");
  page += ESPWebBase::tagInput(FPSTR(typeSubmit), strEmpty, F("Save"));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::tagInput(FPSTR(typeHidden), FPSTR(paramReboot), "1");
  page += F("\n\
</form>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebBase::handleTimeConfig() {
  if (! adminAuthenticate())
    return;

  String script = F("function trimString(field) {\n\
field.value = field.value.trim();\n\
}\n\
function fixNumber(field, minvalue, maxvalue) {\n\
var val = parseInt(field.value);\n\
if (isNaN(val) || (val < minvalue))\n\
field.value = minvalue;\n\
else\n\
if (val > maxvalue)\n\
field.value = maxvalue;\n\
}\n\
function validateForm(form) {\n\
if (form.");
  script += FPSTR(paramNtpServer1);
  script += F(".value == \"\") {\n\
alert(\"NTP server #1 must be set!\");\n\
form.");
  script += FPSTR(paramNtpServer1);
  script += F(".focus();\n\
return false;\n\
}\n\
return true;\n\
}\n\
function updateTime() {\n\
openUrl('");
  script += FPSTR(pathSetTime);
  script += F("?time=' + Math.floor(Date.now() / 1000) + '&dummy=' + Date.now());\n\
}\n\
function refreshData() {\n\
var request = getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathGetTime);
  script += F("?dummy=' + Date.now(), true);\n\
request.onreadystatechange = function() {\n\
if (request.readyState == 4) {\n\
var data = JSON.parse(request.responseText);\n\
if (data.");
  script += FPSTR(jsonUnixTime);
  script += F(" == 0) {\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonDate);
  script += F("').innerHTML = \"Unset\";\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonTime);
  script += F("').innerHTML = \"\";\n\
} else {\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonDate);
  script += F("').innerHTML = data.");
  script += FPSTR(jsonDate);
  script += F(";\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonTime);
  script += F("').innerHTML = data.");
  script += FPSTR(jsonTime);
  script += F(";\n\
}\n\
}\n\
}\n\
request.send(null);\n\
}\n\
setInterval(refreshData, 1000);\n");

  String page = ESPWebBase::webPageStart(F("Time Setup"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<form name=\"time\" method=\"GET\" action=\"");
  page += FPSTR(pathStore);
  page += F("\" onsubmit=\"return validateForm(this)\">\n\
<h3>Time Setup</h3>\n\
Current date and time: <span id=\"");
  page += FPSTR(jsonDate);
  page += F("\"></span> <span id=\"");
  page += FPSTR(jsonTime);
  page += F("\"></span>\n\
<p>\n\
<label>NTP server #1:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramNtpServer1), _ntpServer1, F("onblur=\"trimString(this)\""), MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("\n*<br/>\n\
<label>NTP server #2:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramNtpServer2), _ntpServer2, F("onblur=\"trimString(this)\""), MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("\n<br/>\n\
<label>NTP server #3:</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramNtpServer3), _ntpServer3, F("onblur=\"trimString(this)\""), MAX_STRING_LEN - 1, MAX_STRING_LEN - 1);
  page += F("\n<br/>\n\
<label>Time zone:</label><br/>\n\
<select name=\"");
  page += FPSTR(paramNtpTimeZone);
  page += F("\" size=1>\n");

  for (int8_t i = -11; i <= 13; i++) {
    page += F("<option value=\"");
    page += String(i);
    page += charQuote;
    if (_ntpTimeZone == i)
      page += F(" selected");
    page += charGreater;
    page += F("GMT");
    if (i > 0)
      page += '+';
    page += String(i);
    page += F("</option>\n");
  }
  page += F("</select>\n\
<br/>\n\
<label>Update interval (in sec.):</label><br/>\n");
  page += ESPWebBase::tagInput(FPSTR(typeText), FPSTR(paramNtpUpdateInterval), String(_ntpUpdateInterval / 1000), F("onblur=\"fixNumber(this, 0, 4200000)\""), 10, 10);
  page += F("\n<p>\n");
  page += ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Update time from browser"), F("onclick='updateTime()'"));
  page += F("\n<p>\n");
  page += ESPWebBase::tagInput(FPSTR(typeSubmit), strEmpty, F("Save"));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::tagInput(FPSTR(typeHidden), FPSTR(paramReboot), "1");
  page += F("\n\
</form>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebBase::handleStoreConfig() {
  String argName, argValue;

  for (uint8_t i = 0; i < httpServer->args(); ++i) {
    argName = httpServer->argName(i);
    argValue = httpServer->arg(i);
    setConfigParam(argName, argValue);
  }

  uint16_t offset = 0;
  bool success;

  success = writeConfig(offset);

  String page = ESPWebBase::webPageStart(F("Store Setup"));
  page += F("<meta http-equiv=\"refresh\" content=\"5;URL=/\">\n");
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageBody();
  if (success) {
    page += F("Configuration stored successfully.\n");
    if (httpServer->arg(FPSTR(paramReboot)) == "1")
      page += F("<br/>\n\
<i>You must reboot module to apply new configuration!</i>\n");
    page += F("<p>\n\
Wait for 5 sec. or click <a href=\"/\">this</a> to return to main page.\n");
  } else {
    page += F("Configuration stored unsuccessfully!\n");
  }
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebBase::handleLog() {
  if (! userAuthenticate())
    return;

  String script = F("function clearLog() {\n\
openUrl('");
  script += FPSTR(pathClearLog);
  script += F("?dummy=' + Date.now());\n\
}\n");

  String page = ESPWebBase::webPageStart(F("Log View"));
  page += F("<META http-equiv=\"refresh\" content=\"5;URL=\">\n");
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<h3>Log View</h3>\n\
<textarea cols=\"80\" rows=\"25\" readonly id=\"");
  page += FPSTR(jsonLog);
  page += F("\">\n");
  page += _log->encodeStr(_log->text());
  page += F("</textarea>\n\
<p>\n");
  page += ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Clear!"), F("onclick=\"if (confirm('Are you sure to clear log?') == true) clearLog()\""));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebBase::handleClearLog() {
  _log->clear();

  httpServer->send(200, FPSTR(textHtml), strEmpty);
}

void ESPWebBase::handleGetTime() {
  uint32_t now = getTime();
  String page;

  page += charOpenBrace;
  page += charQuote;
  page += FPSTR(jsonUnixTime);
  page += F("\":");
  page += String(now);
  if (now) {
    int8_t hh, mm, ss;
    uint8_t wd;
    int8_t d, m;
    int16_t y;

    parseUnixTime(now, hh, mm, ss, wd, d, m, y);
    page += F(",\"");
    page += FPSTR(jsonDate);
    page += F("\":\"");
    page += dateToStr(d, m, y);
    page += F("\",\"");
    page += FPSTR(jsonTime);
    page += F("\":\"");
    page += timeToStr(hh, mm, ss);
    page += charQuote;
  }
  page += charCloseBrace;

  httpServer->send(200, FPSTR(textJson), page);
}

void ESPWebBase::handleSetTime() {
  setTime(_max(0, httpServer->arg(FPSTR(paramTime)).toInt()) + _ntpTimeZone * 3600);

  httpServer->send(200, FPSTR(textHtml), strEmpty);
}

void ESPWebBase::handleReboot() {
  String page = ESPWebBase::webPageStart(F("Reboot"));
  page += F("<meta http-equiv=\"refresh\" content=\"5;URL=/\">\n");
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageBody();
  page += F("Rebooting...\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);

  delay(500);
  reboot();
}

void ESPWebBase::handleData() {
  String page;

  page += charOpenBrace;
  page += jsonData();
  page += charCloseBrace;

  httpServer->send(200, FPSTR(textJson), page);
}

String ESPWebBase::jsonData() {
  String result;

  result += charQuote;
  result += FPSTR(jsonFreeHeap);
  result += F("\":");
  result += String(ESP.getFreeHeap());
  result += F(",\"");
  result += FPSTR(jsonUptime);
  result += F("\":");
  result += String(millis() / 1000);
  if (WiFi.getMode() == WIFI_STA) {
    result += F(",\"");
    result += FPSTR(jsonRSSI);
    result += F("\":");
    result += String(WiFi.RSSI());
  }

  return result;
}

String ESPWebBase::btnBack() {
//  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Back"), F("onclick=\"history.back()\""));
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Back"), F("onclick=\"location.href='/'\""));
  result += charLF;

  return result;
}

String ESPWebBase::btnWiFiConfig() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("WiFi Setup"), String(F("onclick=\"location.href='")) + String(FPSTR(pathWiFi)) + String(F("'\"")));
  result += charLF;

  return result;
}

String ESPWebBase::btnTimeConfig() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Time Setup"), String(F("onclick=\"location.href='")) + String(FPSTR(pathTime)) + String(F("'\"")));
  result += charLF;

  return result;
}

String ESPWebBase::btnLog() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Log View"), String(F("onclick=\"location.href='")) + String(FPSTR(pathLog)) + String(F("'\"")));
  result += charLF;

  return result;
}

String ESPWebBase::btnReboot() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Reboot!"), String(F("onclick=\"if (confirm('Are you sure to reboot?')) location.href='")) + String(FPSTR(pathReboot)) + String(F("'\"")));
  result += charLF;

  return result;
}

String ESPWebBase::navigator() {
  String result = btnWiFiConfig();
  result += btnTimeConfig();
  result += btnLog();
  result += btnReboot();

  return result;
}

String ESPWebBase::getContentType(const String &fileName) {
  if (httpServer->hasArg(F("download")))
    return String(F("application/octet-stream"));
  else if (fileName.endsWith(F(".htm")) || fileName.endsWith(F(".html")))
    return String(FPSTR(textHtml));
  else if (fileName.endsWith(F(".css")))
    return String(FPSTR(textCss));
  else if (fileName.endsWith(F(".js")))
    return String(FPSTR(applicationJavascript));
  else if (fileName.endsWith(F(".png")))
    return String(F("image/png"));
  else if (fileName.endsWith(F(".gif")))
    return String(F("image/gif"));
  else if (fileName.endsWith(F(".jpg")) || fileName.endsWith(F(".jpeg")))
    return String(F("image/jpeg"));
  else if (fileName.endsWith(F(".ico")))
    return String(F("image/x-icon"));
  else if (fileName.endsWith(F(".xml")))
    return String(F("text/xml"));
  else if (fileName.endsWith(F(".pdf")))
    return String(F("application/x-pdf"));
  else if (fileName.endsWith(F(".zip")))
    return String(F("application/x-zip"));
  else if (fileName.endsWith(F(".gz")))
    return String(F("application/x-gzip"));

  return String(FPSTR(textPlain));
}

bool ESPWebBase::handleFileRead(const String &path) {
  String fileName = path;
  if (fileName.endsWith(strSlash))
    fileName += FPSTR(indexHtml);
  String contentType = getContentType(fileName);
  String pathWithGz = fileName + F(".gz");
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(fileName)) {
    if (SPIFFS.exists(pathWithGz))
      fileName = pathWithGz;
    File file = SPIFFS.open(fileName, "r");
    size_t sent = httpServer->streamFile(file, contentType);
    file.close();

    return true;
  }

  return false;
}

String ESPWebBase::webPageStart(const String &title) {
  String result = FPSTR(headerTitleOpen);
  result += title;
  result += FPSTR(headerTitleClose);

  return result;
}

String ESPWebBase::webPageStyle(const String &style, bool file) {
  String result;

  if (file) {
    result = FPSTR(headerStyleExtOpen);
    result += style;
    result += FPSTR(headerStyleExtClose);
  } else {
    result = FPSTR(headerStyleOpen);
    result += style;
    result += FPSTR(headerStyleClose);
  }

  return result;
}

String ESPWebBase::webPageScript(const String &script, bool file) {
  String result;

  if (file) {
    result = FPSTR(headerScriptExtOpen);
    result += script;
    result += FPSTR(headerScriptExtClose);
  } else {
    result = FPSTR(headerScriptOpen);
    result += script;
    result += FPSTR(headerScriptClose);
  }

  return result;
}

String ESPWebBase::webPageBody() {
  String result = FPSTR(headerBodyOpen);
  result += charGreater;
  result += charLF;

  return result;
}

String ESPWebBase::webPageBody(const String &extra) {
  String result = FPSTR(headerBodyOpen);
  result += charSpace;
  result += extra;
  result += charGreater;
  result += charLF;

  return result;
}

String ESPWebBase::webPageEnd() {
  String result = FPSTR(footerBodyClose);

  return result;
}

String ESPWebBase::escapeQuote(const String &str) {
  String result;
  int start = 0, pos;

  while (start < str.length()) {
    pos = str.indexOf(charQuote, start);
    if (pos != -1) {
      result += str.substring(start, pos) + F("&quot;");
      start = pos + 1;
    } else {
      result += str.substring(start);
      break;
    }
  }

  return result;
}

String ESPWebBase::tagInput(const String &type, const String &name, const String &value, int16_t size, int16_t maxlength) {
  String result = FPSTR(inputTypeOpen);

  result += type;
  result += charQuote;
  if (name != strEmpty) {
    result += FPSTR(inputNameOpen);
    result += name;
    result += charQuote;
  }
  if (value != strEmpty) {
    result += FPSTR(inputValueOpen);
    result += ESPWebBase::escapeQuote(value);
    result += charQuote;
  }
  if (size >= 0) {
    result += FPSTR(inputSize);
    result += String(size);
  }
  if (maxlength >= 0) {
    result += FPSTR(inputMaxLength);
    result += String(maxlength);
  }
  result += charGreater;

  return result;
}

String ESPWebBase::tagInput(const String &type, const String &name, const String &value, const String &extra, int16_t size, int16_t maxlength) {
  String result = FPSTR(inputTypeOpen);

  result += type;
  result += charQuote;
  if (name != strEmpty) {
    result += FPSTR(inputNameOpen);
    result += name;
    result += charQuote;
  }
  if (value != strEmpty) {
    result += FPSTR(inputValueOpen);
    result += ESPWebBase::escapeQuote(value);
    result += charQuote;
  }
  if (size >= 0) {
    result += FPSTR(inputSize);
    result += String(size);
  }
  if (maxlength >= 0) {
    result += FPSTR(inputMaxLength);
    result += String(maxlength);
  }
  result += charSpace;
  result += extra;
  result += charGreater;

  return result;
}

#ifdef LED_PIN
void ESPWebBase::pulseLed() {
  const uint8_t minBrightness = 255;
  const uint8_t maxBrightness = 0;
  const int8_t defDelta = -8;

  static int8_t delta = defDelta;
  static int16_t brightness = minBrightness;

  analogWrite(LED_PIN, brightness);
  if (_pulse == PULSE) {
    if (delta != defDelta) {
      if (brightness == minBrightness)
        brightness = maxBrightness;
      else
        brightness = minBrightness;
    }
    delta = -delta;
  } else if (_pulse == FASTPULSE) {
    if (brightness == minBrightness)
      brightness = maxBrightness;
    else
      brightness = minBrightness;
  } else if (_pulse == BREATH) {
    brightness += delta;
    if ((brightness < maxBrightness) || (brightness > minBrightness)) {
      delta = -delta;
      brightness = constrain(brightness, maxBrightness, minBrightness);
    }
  } else if (_pulse == FADEIN) {
    brightness -= abs(delta);
    if (brightness < maxBrightness) {
      brightness = minBrightness;
    }
  } else if (_pulse == FADEOUT) {
    brightness -= abs(delta);
    if (brightness > minBrightness) {
      brightness = maxBrightness;
    }
  }
}
#endif
