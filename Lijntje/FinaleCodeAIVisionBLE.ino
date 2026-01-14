/*
  XIAO ESP32S3 (BLE client) -> Petoi Bittle X (BiBoard) via BLE UART (NUS)

  - Scans for a device named "BittleDC_BLE"
  - Connects to it
  - Finds Nordic UART Service (NUS)
  - Writes Serial Monitor lines to the robot (RX characteristic)
  - Prints robot notifications (TX characteristic)

  If the robot connects but ignores commands, change:
    WRITE_WITH_RESPONSE and/or APPEND_NEWLINE
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Seeed_Arduino_SSCMA.h>

SSCMA AI;

static const char* TARGET_NAME = "ogintevullen_SSP";

// Nordic UART Service (NUS) UUIDs used by Petoi BLE-UART
static BLEUUID NUS_SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID NUS_RX_UUID     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // Write: client -> robot
static BLEUUID NUS_TX_UUID     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // Notify: robot -> client

// If robot ignores commands, try toggling these:
static const bool WRITE_WITH_RESPONSE = false; // try true if needed
static const bool APPEND_NEWLINE      = false; // try true if needed

static int scanTime = 5; // seconds
static BLEScan* pBLEScan = nullptr;

static bool doConnect = false;
static bool connected = false;
static BLEAddress* pServerAddress = nullptr;

static BLEClient* pClient = nullptr;
static BLERemoteCharacteristic* pRxChar = nullptr;
static BLERemoteCharacteristic* pTxChar = nullptr;

// ---- Notification callback (robot -> XIAO) ----
static void notifyCallback(
  BLERemoteCharacteristic* /*pBLERemoteCharacteristic*/,
  uint8_t* pData,
  size_t length,
  bool /*isNotify*/
) {
  Serial.print("[Bittle] ");
  for (size_t i = 0; i < length; i++) Serial.print((char)pData[i]);
  Serial.println();
}

// ---- Client connection callbacks ----
class MyClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* /*pclient*/) override {
      connected = true;
      Serial.println("BLE connected.");
    }

    void onDisconnect(BLEClient* /*pclient*/) override {
      connected = false;
      Serial.println("BLE disconnected.");
    }
};

// ---- Scan callbacks ----
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
      // Print what you see (you already confirmed this works)
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());

      // Match by name (exact match)
      if (advertisedDevice.haveName() && advertisedDevice.getName() == std::string(TARGET_NAME)) {
        Serial.printf("Found target %s, stopping scan.\n", TARGET_NAME);

        // Save address for connection
        if (pServerAddress) {
          delete pServerAddress;
          pServerAddress = nullptr;
        }
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());

        doConnect = true;
        pBLEScan->stop();
      }
    }
};

// ---- Connect + discover NUS service/chars ----
static bool connectToServer() {
  if (!pServerAddress) return false;

  Serial.printf("Connecting to: %s\n", pServerAddress->toString().c_str());

  // Create client if needed
  if (!pClient) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks());
  }

  if (!pClient->connect(*pServerAddress)) {
    Serial.println("Connect failed.");
    return false;
  }

  // Discover service
  BLERemoteService* pService = pClient->getService(NUS_SERVICE_UUID);
  if (pService == nullptr) {
    Serial.println("NUS service not found. Disconnecting.");
    pClient->disconnect();
    return false;
  }

  // Discover characteristics
  pRxChar = pService->getCharacteristic(NUS_RX_UUID);
  pTxChar = pService->getCharacteristic(NUS_TX_UUID);

  if (pRxChar == nullptr) {
    Serial.println("NUS RX characteristic not found. Disconnecting.");
    pClient->disconnect();
    return false;
  }
  if (pTxChar == nullptr) {
    Serial.println("NUS TX characteristic not found. Disconnecting.");
    pClient->disconnect();
    return false;
  }

  // Subscribe to notifications from robot
  if (pTxChar->canNotify()) {
    pTxChar->registerForNotify(notifyCallback);
    Serial.println("Subscribed to TX notifications.");
  } else {
    Serial.println("TX characteristic cannot notify (sending may still work).");
  }

  Serial.println("Ready. Type Petoi commands (e.g., ksit, kwkF) in Serial Monitor and press Enter.");
  return true;
}

// ---- Send a command to the robot ----
static void sendCommandLine(const String& lineIn) {
  if (!connected || !pClient || !pClient->isConnected() || !pRxChar) return;

  String line = lineIn;
  line.trim();
  if (line.length() == 0) return;

  if (APPEND_NEWLINE) line += "\n";

  std::string s = line.c_str();
  Serial.print(">> ");
  Serial.println(line);

  // writeValue(data, length, response)
  pRxChar->writeValue((uint8_t*)s.data(), s.size(), WRITE_WITH_RESPONSE);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  AI.begin();
  Serial.println("Scanning...");

  BLEDevice::init(""); // no local name needed

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
}

void loop() {
  // If not connected, scan (or rescan)
  if (!connected && !doConnect) {
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    Serial.print("Devices found: ");
    Serial.println(foundDevices.getCount());
    Serial.println("Scan done!");
    pBLEScan->clearResults();
    delay(250);
  }

  // If scan found target, connect once
  if (doConnect) {
    doConnect = false;
    if (connectToServer()) {
      // connected flag is set by callback
    } else {
      // If failed, retry scanning shortly
      Serial.println("Will retry scan/connect...");
      delay(1000);
    }
  }

  // If connected, forward Serial Monitor lines to the robot
  //  if (connected && Serial.available()) {
  //    String line = Serial.readStringUntil('\n');
  //    sendCommandLine(line);
  //  }

  // AI Vision Control - NIEUWE TARGET MAPPING
  if (connected && !AI.invoke())
  { 
    for (int i = 0; i < AI.classes().size(); i++)
    {
      if (AI.classes()[i].score > 80) {
          if (AI.classes()[i].target == 0) {
            Serial.println("VOORUIT");
            sendCommandLine("kwkF");
            delay(3000);
          }
          if (AI.classes()[i].target == 1) {
            Serial.println("PUSHUPS");
            sendCommandLine("kpu");
            delay(5000);
          }
          if (AI.classes()[i].target == 2) {
            Serial.println("CROUCH");
            sendCommandLine("kcr");
            delay(3000);
          }
          if (AI.classes()[i].target == 3) {
            Serial.println("TRAP");
            sendCommandLine("ktrap oplopen");  // Tijdelijk sit, vervang later met eigen commando
            delay(3000);
          }
          if (AI.classes()[i].target == 4) {
            Serial.println("KICK");
            sendCommandLine("kck");
            delay(3000);
          }
          if (AI.classes()[i].target == 5) {
            Serial.println("LINKS 90");
            sendCommandLine("kvtL 90");
            delay(3000);
          }
          if (AI.classes()[i].target == 6) {
            Serial.println("DANS");
            sendCommandLine("kup");  // Tijdelijk table, vervang later met eigen commando
            delay(4000);
          }
      }
    }
  }

  // If disconnected, clear stale pointers and allow scan again
  if (pClient && !pClient->isConnected()) {
    connected = false;
    pRxChar = nullptr;
    pTxChar = nullptr;
    delay(500);
  }

  delay(5);
}
