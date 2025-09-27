#define VLARE_CLASSIC_BLUETOOTH
// #define VLARE_BLE

#include "Preferences.h"
#include "LittleFS.h"

#if defined(VLARE_CLASSIC_BLUETOOTH)
#include "BluetoothSerial.h"
#else
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#endif

#define PACKET_HEADER 0xA5
#define PACKET_FOOTER 0x5A
#define PACKET_BUFFER_SIZE 256
#define MAX_FILENAME_LEN 16

#define CMD_CONNECT_WIFI 0x10
#define CMD_SEND_WIFI_PASS 0x11
#define CMD_CONFIRM_WIFI_CONN 0x12
#define CMD_GET_STATUS 0x13
#define CMD_START_FILE_TRANSFER 0x14
#define CMD_END_FILE_TRANSFER 0x15
#define CMD_SET_PICK_VALUE 0x16
#define CMD_SET_RPM 0x17
#define CMD_SET_DEVICE_NAME 0x18

#define SUCCESS 0x00
#define LENGTH_MISMATCH 0x01
#define CRC_CHECK_FAILED 0x02
#define ERROR 0x03

char bodyFileName[MAX_FILENAME_LEN + 1] = "No File";
uint32_t bodyTotalPicks = 0;
uint32_t bodyCurrentPicks = 0;
char borderFileName[MAX_FILENAME_LEN + 1] = "No File";
uint32_t borderTotalPicks = 0;
uint32_t borderCurrentPicks = 0;
String fullDeviceName;
byte packetBuffer[PACKET_BUFFER_SIZE];
uint16_t currentRPM = 0;

bool isReceivingFile = false;
File uploadFile;
uint32_t expectedFileSize = 0;
uint32_t bytesReceived = 0;

Preferences preferences;
#if defined(VLARE_CLASSIC_BLUETOOTH)
BluetoothSerial SerialBT;
#else
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#endif


void processIncomingPacket(byte* packetData, size_t len);
void sendBluetoothData(const byte* data, size_t len);

byte calculateCRC(const byte* data, byte len) {
  uint32_t sum = 0;
  for (int i = 0; i < len; i++) { sum += data[i]; }
  return (byte)(sum & 0xFF);
}

void sendResponsePacket(byte command, const byte* data, byte len) {
  int packetLen = 5 + len;
  byte txPacket[packetLen];
  txPacket[0] = PACKET_HEADER;
  txPacket[1] = command;
  txPacket[2] = len;
  if (len > 0) { memcpy(&txPacket[3], data, len); }
  txPacket[3 + len] = calculateCRC(data, len);
  txPacket[4 + len] = PACKET_FOOTER;
  sendBluetoothData(txPacket, packetLen);
}

void processIncomingPacket(byte* packetData, size_t len) {
  byte command = packetData[1];
  byte dataLen = packetData[2];
  byte* data = &packetData[3];
  Serial.print("Processing Command: 0x");
  Serial.println(command, HEX);

  // 1. Basic Validation: Header, Footer, and Minimum Length (Header, Cmd, Len, CRC, Footer)
  if (len < 5 || packetData[0] != PACKET_HEADER || packetData[len - 1] != PACKET_FOOTER) {
    Serial.println("Invalid packet structure (Header/Footer/Length).");
    return;
  }


  byte receivedCRC = packetData[3 + dataLen];

  // 2. Data Length Validation
  // The total length should be Header(1) + Cmd(1) + Len(1) + Data(dataLength) + CRC(1) + Footer(1)
  if (len != dataLen + 5) {
    Serial.println("Packet length mismatch.");
    byte errorData[] = { LENGTH_MISMATCH };
    sendResponsePacket(command, errorData, 1);
    return;
  }

  // 3. CRC Validation
  byte calculated_crc = calculateCRC(data, dataLen);
  if (calculated_crc != receivedCRC) {
    Serial.printf("CRC mismatch! Received: 0x%02X, Calculated: 0x%02X\n", receivedCRC, calculated_crc);
    byte errorData[] = { CRC_CHECK_FAILED };
    sendResponsePacket(command, errorData, 1);
    return;
  }

  Serial.printf("Packet Validated! Command: 0x%02X\n", command);


  switch (command) {
    case CMD_GET_STATUS:
      {
        Serial.println("Received GET_STATUS command.");
        const int responseLen = 48;
        byte statusData[responseLen];
        memset(statusData, 0, responseLen);
        memcpy(statusData, &bodyTotalPicks, 4);
        memcpy(statusData + 4, &bodyCurrentPicks, 4);
        memcpy(statusData + 8, bodyFileName, strlen(bodyFileName));
        memcpy(statusData + 24, &borderTotalPicks, 4);
        memcpy(statusData + 28, &borderCurrentPicks, 4);
        memcpy(statusData + 32, borderFileName, strlen(borderFileName));
        sendResponsePacket(CMD_GET_STATUS, statusData, responseLen);
        break;
      }
    case CMD_START_FILE_TRANSFER:
      {
        Serial.println("Received START_FILE_TRANSFER command.");
        if (dataLen < 6) {
          Serial.printf("ERROR: Invalid data length for START_FILE_TRANSFER. Expected >= 6, got %d\n", dataLen);
          byte errorData[] = { LENGTH_MISMATCH };  // Error: Bad length
          sendResponsePacket(CMD_START_FILE_TRANSFER, errorData, 1);
          break;
        }
        // Calculate the dynamic file name length
        // N = dataLen - 1 (Type) - 4 (Size)
        int fileNameLen = dataLen - 5;

        if (fileNameLen > MAX_FILENAME_LEN) {
          Serial.printf("ERROR: Filename is too long. Max %d, got %d\n", MAX_FILENAME_LEN, fileNameLen);
          byte errorData[] = { LENGTH_MISMATCH };
          sendResponsePacket(CMD_START_FILE_TRANSFER, errorData, 1);
          break;
        }

        byte fileType = data[0];
        char receivedFileName[MAX_FILENAME_LEN + 1];

        memcpy(receivedFileName, &data[1], fileNameLen);
        receivedFileName[fileNameLen] = '\0';
        memcpy(&expectedFileSize, &data[1 + fileNameLen], 4);
        String filePath;
        if (fileType == 0x01) {
          filePath = "/body.bmp";
          strcpy(bodyFileName, receivedFileName);
          bodyTotalPicks = expectedFileSize;
        } else if (fileType == 0x02) {
          filePath = "/border.bmp";
          strcpy(borderFileName, receivedFileName);
          borderTotalPicks = expectedFileSize;
        } else {
          Serial.printf("ERROR: Unknown file type 0x%02X\n", fileType);
          byte errorData[] = { ERROR };
          sendResponsePacket(CMD_START_FILE_TRANSFER, errorData, 1);
          break;
        }
        Serial.printf("Starting file upload for: %s (%d bytes)\n", receivedFileName, expectedFileSize);
        uploadFile = LittleFS.open(filePath, "w");
        if (!uploadFile) {
          Serial.println("ERROR: Failed to open file for writing!");
          byte errorData[] = { ERROR };
          sendResponsePacket(CMD_START_FILE_TRANSFER, errorData, 1);
          break;
        }
        bytesReceived = 0;
        isReceivingFile = true;
        byte successData[] = { SUCCESS };
        sendResponsePacket(CMD_START_FILE_TRANSFER, successData, 1);
        break;
      }
    case CMD_END_FILE_TRANSFER:
      {
        Serial.println("Received END_FILE_TRANSFER command.");
        if (isReceivingFile) {
          uploadFile.close();
          isReceivingFile = false;
          Serial.printf("File transfer ended by command. Received %d/%d bytes.\n", bytesReceived, expectedFileSize);
        }
        byte successData[] = { SUCCESS };
        sendResponsePacket(CMD_END_FILE_TRANSFER, successData, 1);
        break;
      }
    case CMD_SET_PICK_VALUE:
      {
        Serial.println("Received SET_PICK_VALUE command.");
        // Expected data: 1 byte for type (Body/Border) + 4 bytes for value
        if (dataLen != 5) {
          Serial.printf("ERROR: Invalid data length for SET_PICK_VALUE. Expected 5, got %d\n", dataLen);
          byte errorData[] = { LENGTH_MISMATCH };
          sendResponsePacket(CMD_SET_PICK_VALUE, errorData, 1);
          break;
        }

        byte pickType = data[0];
        uint32_t newPickValue;
        memcpy(&newPickValue, &data[1], 4);  // Safely copy 4 bytes into the uint32_t

        if (pickType == 0x01) {  // 0x01 for Body
          bodyCurrentPicks = newPickValue;
          Serial.printf("Body picks set to %u\n", newPickValue);
          byte successData[] = { SUCCESS };
          sendResponsePacket(CMD_SET_PICK_VALUE, successData, 1);
        } else if (pickType == 0x02) {  // 0x02 for Border
          borderCurrentPicks = newPickValue;
          Serial.printf("Border picks set to %u\n", newPickValue);
          byte successData[] = { SUCCESS };
          sendResponsePacket(CMD_SET_PICK_VALUE, successData, 1);
        } else {
          Serial.printf("ERROR: Unknown pick type 0x%02X\n", pickType);
          byte errorData[] = { ERROR };
          sendResponsePacket(CMD_SET_PICK_VALUE, errorData, 1);
        }
        break;
      }

    case CMD_SET_RPM:
      {
        Serial.println("Received SET_RPM command.");
        // Expected data: 2 bytes for the RPM value (uint16_t)
        if (dataLen != 2) {
          Serial.printf("ERROR: Invalid data length for SET_RPM. Expected 2, got %d\n", dataLen);
          byte errorData[] = { LENGTH_MISMATCH };
          sendResponsePacket(CMD_SET_RPM, errorData, 1);
          break;
        }

        uint16_t newRPM;
        memcpy(&newRPM, data, 2);  // Safely copy 2 bytes into the uint16_t

        currentRPM = newRPM;  // Assumes you have a global 'uint16_t currentRPM;'

        // Optional: Save the RPM value so it persists after a reboot
        preferences.begin("vlare-loom", false);
        preferences.putUShort("rpmValue", newRPM);
        preferences.end();

        Serial.printf("RPM set to %u and saved.\n", currentRPM);

        byte successData[] = { SUCCESS };
        sendResponsePacket(CMD_SET_RPM, successData, 1);
        break;
      }
    case CMD_SET_DEVICE_NAME:
      {
        Serial.println("Received SET_DEVICE_NAME command.");
        char newName[dataLen + 1];
        memcpy(newName, data, dataLen);
        newName[dataLen] = '\0';
        preferences.begin("vlare-loom", false);
        preferences.putString("customName", newName);
        preferences.end();
        Serial.printf("Device name updated to: %s\n", newName);
        byte successData[] = { SUCCESS };
        sendResponsePacket(CMD_SET_DEVICE_NAME, successData, 1);
        Serial.println("Rebooting to apply new name...");
        delay(100);
        ESP.restart();
        break;
      }
  }
}

#if defined(VLARE_CLASSIC_BLUETOOTH)
void bt_status_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.printf("\n***********************\nClient Connected!\nDevice Name: %s\n***********************\n", fullDeviceName.c_str());
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("\n***********************\nClient Disconnected!\n***********************");
    if (isReceivingFile) {
      uploadFile.close();
      isReceivingFile = false;
      Serial.println("File transfer cancelled due to disconnect.");
    }
  }
}
#else
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.printf("\n***********************\nBLE Client Connected!\nDevice Name: %s\n***********************\n", fullDeviceName.c_str());
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("\n***********************\nBLE Client Disconnected!\n***********************");
    if (isReceivingFile) {
      uploadFile.close();
      isReceivingFile = false;
      Serial.println("File transfer cancelled due to disconnect.");
    }
    pServer->getAdvertising()->start();
  }
};
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    size_t len = pCharacteristic->getLength();
    if (len > 0) {
      uint8_t* data_ptr = pCharacteristic->getData();
      if (data_ptr[0] == PACKET_HEADER && data_ptr[len - 1] == PACKET_FOOTER) {
        processIncomingPacket(data_ptr, len);
      } else {
        if (isReceivingFile) {
          uploadFile.write(data_ptr, len);
          bytesReceived += len;
          if (bytesReceived >= expectedFileSize) {
            uploadFile.close();
            isReceivingFile = false;
            Serial.printf("\n---------------------------------\nFile transfer completed successfully!\nTotal bytes received: %d\nSwitching back to Command Mode.\n---------------------------------\n", bytesReceived);
          }
        }
      }
    }
  }
};
#endif

void setupBluetooth() {
#if defined(VLARE_CLASSIC_BLUETOOTH)
  SerialBT.register_callback(bt_status_callback);
  if (!SerialBT.begin(fullDeviceName)) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.printf("Classic Bluetooth device \"%s\" is ready.\n", fullDeviceName.c_str());
  }
#else
  BLEDevice::init(fullDeviceName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.printf("BLE device \"%s\" is advertising.\n", fullDeviceName.c_str());
#endif
}

void handleBluetooth() {
#if defined(VLARE_CLASSIC_BLUETOOTH)
  if (isReceivingFile) {
    if (SerialBT.available()) {

      size_t bytesToRead = min((size_t)SerialBT.available(), (size_t)PACKET_BUFFER_SIZE);
      size_t bytesRead = SerialBT.readBytes(packetBuffer, bytesToRead);
      uploadFile.write(packetBuffer, bytesRead);
      bytesReceived += bytesRead;
      if (bytesReceived >= expectedFileSize) {
        uploadFile.close();
        isReceivingFile = false;
        Serial.printf("\n----------------------------------------\nFile transfer completed successfully!\nTotal bytes received: %d\nSwitching back to Command Mode.\n----------------------------------------\n", bytesReceived);
      }
    }
  } else {
    static int bufferIndex = 0;
    static int expectedPacketLength = 0;
    while (SerialBT.available()) {
      byte inByte = SerialBT.read();
      if (bufferIndex == 0) {
        if (inByte == PACKET_HEADER) { packetBuffer[bufferIndex++] = inByte; }
        continue;
      }
      packetBuffer[bufferIndex++] = inByte;
      if (bufferIndex == 3) {
        expectedPacketLength = 5 + packetBuffer[2];
        if (expectedPacketLength > PACKET_BUFFER_SIZE) {
          bufferIndex = 0;
          continue;
        }
      }
      if (expectedPacketLength > 0 && bufferIndex == expectedPacketLength) {
        if (packetBuffer[bufferIndex - 1] == PACKET_FOOTER) {
          processIncomingPacket(packetBuffer, bufferIndex);
        } else {
          Serial.printf("ERROR: Packet footer incorrect. Expected 0x5A, got 0x%02X.\n", packetBuffer[bufferIndex - 1]);
        }
        bufferIndex = 0;
      }
    }
  }
#else
  // BLE is handled entirely by callbacks
#endif
}

void sendBluetoothData(const byte* data, size_t len) {
  Serial.print("TX -> ");
  for (int i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();
#if defined(VLARE_CLASSIC_BLUETOOTH)
  SerialBT.write(data, len);
#else
  if (deviceConnected) {
    pCharacteristic->setValue((uint8_t*)data, len);
    pCharacteristic->notify();
  }
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- VLARE Loom Firmware ---");
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL ERROR: Failed to mount LittleFS");
    while (1)
      ;
  }
  preferences.begin("vlare-loom", true);
  String customName = preferences.getString("customName", "Device");
  preferences.end();
  fullDeviceName = "VLARE_" + customName;
  setupBluetooth();
}

void loop() {
  handleBluetooth();
}