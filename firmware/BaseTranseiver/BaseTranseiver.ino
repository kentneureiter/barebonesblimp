#include <esp_now.h>
#include <WiFi.h>

typedef struct ControlInput {
    float params[13]; //FIXME magic number
} ControlInput;


typedef struct ReceivedData {
    int flag;
    float values[6];  //FIXME magic number
} ReceivedData;



#define MAX_FLAGS 6
float storedData[MAX_FLAGS][6] = {0};  // Data storage
bool flagReceived[MAX_FLAGS] = {false};  // Keep track of received flags

// Adjust these values as needed for your specific application
constexpr size_t CONTROL_PARAMS_SIZE = sizeof(float) * 13; // Size of control parameters
constexpr size_t MAC_ADDRESS_SIZE = 6; // MAC address size

ReceivedData latestReceivedData;


// // Structure to hold control parameters
// struct ControlParams {
//   float params[13]; // Example: 13 control parameters
// };
const int MAX_RECEIVERS = 15;      
const int MAC_LENGTH = 6;          

uint8_t slaveAddresses[MAX_RECEIVERS][MAC_LENGTH] = {};
uint8_t BROADCAST_MAC[MAC_LENGTH] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// Function declarations
void onDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
void onDataSend(const uint8_t *mac, esp_now_send_status_t status);
void addPeer(const uint8_t *peerAddr);
void removePeer(const uint8_t *peerAddr);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(200);
  
  Serial.println("Transmitter ESP Board");
  Serial.println(WiFi.macAddress());
  Serial.println("Do not use this MAC.");

  esp_now_deinit();
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW initialized");
  }



  esp_now_register_send_cb(onDataSend);
  esp_now_register_recv_cb(onDataReceive);

  addPeer(BROADCAST_MAC);
}

void loop() {
  
  // Listening for incoming serial data indicating MAC addresses to add/remove peers
  while (Serial.available()) {
    char type = Serial.read(); // Read the type of operation
    uint8_t peerAddr[MAC_ADDRESS_SIZE];
    if (type == 'A') { // Add peer
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        addPeer(peerAddr);
      }
    } else if (type == 'R') { // Remove peer
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        removePeer(peerAddr);
      }
    } else if (type == 'C') { // Control command
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        ControlInput params;
        if (Serial.readBytes((char*)&params, CONTROL_PARAMS_SIZE) == CONTROL_PARAMS_SIZE) {
          esp_now_send(peerAddr, (uint8_t*)&params, CONTROL_PARAMS_SIZE);
        }
        Serial.println("Sent Controls!");
      }
    } else if (type == 'D') { // Preference to save
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        static uint8_t buffer[250]; // Adjust based on your expected maximum message size
        size_t bytesToRead = Serial.available(); // Determine how many bytes we have for the preference data
        bytesToRead = min(bytesToRead, sizeof(buffer)); // Ensure we don't read more than the buffer size
        
        size_t bytesRead = Serial.readBytes(buffer, bytesToRead); // Read the preference data into the buffer
        Serial.println("Sent Preferences! ");
        // Serial.println((char*)buffer);
        esp_now_send(peerAddr, buffer, bytesRead); // Send the preference data to the specified peer
      }
    } else if (type == 'G') { // add ground station to drone for feedback
      if (Serial.readBytes(peerAddr, MAC_ADDRESS_SIZE) == MAC_ADDRESS_SIZE) {
        uint8_t groundStationMAC[MAC_ADDRESS_SIZE + 1]; // Array to hold the MAC address plus the prefix byte
        groundStationMAC[0] = 0x70; // Set the first byte to 0x70 as required
        
        // Use WiFi.macAddress() to get the MAC address and store it directly into the array starting from the second position
        WiFi.macAddress(&groundStationMAC[1]); // Fill the array with the MAC address starting from position 1
        
        // Now groundStationMAC contains the prefixed MAC address ready to be sent
        // Assuming esp_now_send(peerAddr, data, len) is used for sending the MAC address
        esp_now_send(peerAddr, groundStationMAC, sizeof(groundStationMAC)); // Send the ground station MAC with the prefix
        
        Serial.println("Sent Ground Station MAC! ");
      }
    } else if (type == 'I') { // retrieve data from ground station to use in python
      if (flagReceived[1]){
        for (int i = 0; i < 6; i++) {
          Serial.write((uint8_t*)&storedData[1][i], sizeof(storedData[1][i]));
        }
        Serial.println(); // End of transmission for easier reading on Python side
      } else {
        Serial.println("No Data.");
      }
    }
  }
}
esp_now_peer_info_t peerInfo;

void addPeer(const uint8_t *peerAddr) {
  memcpy(peerInfo.peer_addr, peerAddr, MAC_ADDRESS_SIZE);
  peerInfo.channel = 1; // Use auto channel
  peerInfo.encrypt = false; // No encryption

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Added peer successfully.");
  }
}

void removePeer(const uint8_t *peerAddr) {
  if (esp_now_del_peer(peerAddr) != ESP_OK) {
    Serial.println("Failed to remove peer");
  } else{
    Serial.println("Removed peer");
  }
}

void onDataSend(const uint8_t *mac, esp_now_send_status_t status) {
  // char ackMessage[32]; // Buffer for the acknowledgement message
  // if (status == ESP_NOW_SEND_SUCCESS) {
  //   snprintf(ackMessage, sizeof(ackMessage), "Send OK to %02x:%02x:%02x:%02x:%02x:%02x\n",
  //            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // } else {
  //   snprintf(ackMessage, sizeof(ackMessage), "Send FAIL to %02x:%02x:%02x:%02x:%02x:%02x\n",
  //            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // }
  // Serial.write(ackMessage);
}

void onDataReceive(const uint8_t *mac, const uint8_t *incomingData, int data_len) {
  
  if (data_len == sizeof(ReceivedData)) {
    memcpy(&latestReceivedData, incomingData, data_len);
    
    if (latestReceivedData.flag >= 0 && latestReceivedData.flag < MAX_FLAGS) {
        for (int i = 0; i < 6; i++) {
            storedData[latestReceivedData.flag][i] = latestReceivedData.values[i];
        }
        flagReceived[latestReceivedData.flag] = true;  // Mark this flag as received
    } else {
        //Serial.println("Error: Invalid flag received.");
    }
  }
}
