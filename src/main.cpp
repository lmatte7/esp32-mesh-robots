#include <Arduino.h>
#include <PS4Controller.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <esp_now.h>

#define LEADER 0

int oneIn = 26;
int twoIn = 25;

int threeIn = 33;
int fourIn = 32;

typedef struct motor_message {
    int motor1;
    int motor2;
} motor_message;

typedef struct leader_state {
  bool motor1Forward;
  bool motor2Forward;
  bool motor1Backward;
  bool motor2Backward;
  bool motor1Stopped;
  bool motor2Stopped;
} leader_state;

motor_message motorData;

leader_state state;

esp_now_peer_info_t peerInfo;

void motor1Forward() {
  digitalWrite(oneIn, HIGH);
  digitalWrite(twoIn, LOW);
}

void motor2Forward() {
  digitalWrite(threeIn, HIGH);
  digitalWrite(fourIn, LOW);
}

void motor1Backward() {
  digitalWrite(oneIn, LOW);
  digitalWrite(twoIn, HIGH);
}

void motor2Backward() {
  digitalWrite(threeIn, LOW);
  digitalWrite(fourIn, HIGH);
}

void motor1Stop() {
  digitalWrite(oneIn, LOW);
  digitalWrite(twoIn, LOW);
}

void motor2Stop() {
  digitalWrite(threeIn, LOW);
  digitalWrite(fourIn, LOW);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("DATA!!!");
  memcpy(&motorData, incomingData, sizeof(motorData));
  Serial.printf("%d",motorData.motor1);
  if(!LEADER) {


    if(motorData.motor1 == 1) {
      motor1Forward();
    } else if (motorData.motor1 == 0) {
      motor1Stop();
    } else if(motorData.motor1 == -1) {
      motor1Backward();
    }
    if(motorData.motor2 == 1) {
      motor2Forward();
    } else if (motorData.motor2 == 0) {
      motor2Stop();
    } else if(motorData.motor2 == -1) {
      motor2Backward();
    }
  }
}

uint8_t broadcastAddress1[] = {0x08, 0xD1, 0xF9, 0x37, 0x71, 0xB0};
uint8_t broadcastAddress2[] = {0x48, 0xE7, 0x29, 0xAF, 0x9C, 0x28};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Set LED as output
  pinMode(oneIn, OUTPUT);
  pinMode(twoIn, OUTPUT);
    
  pinMode(threeIn, OUTPUT);
  pinMode(fourIn, OUTPUT);
    
  // configure LED PWM functionalitites
  
  // Serial monitor setup
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_config_espnow_rate(WIFI_IF_STA,WIFI_PHY_RATE_54M);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  if(LEADER) {
    PS4.begin("D4:25:8B:F9:A7:62");
    Serial.println("Ready.");

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }

    memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }

    // Set leader state to 0
    state.motor1Backward = false;
    state.motor1Forward = false;
    state.motor2Backward = false;
    state.motor2Forward = false;
    state.motor1Stopped = true;
    state.motor2Stopped = true;

  } else {
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);

  }
}



 
void loop()
{
  if(LEADER) {

    if (PS4.isConnected()) {
      if (PS4.LStickY()) {
        if(PS4.LStickY() > 100) {
          motor2Forward();
          if(!state.motor2Forward) {
            motorData.motor2 = 1;
            esp_err_t result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
            state.motor2Forward = true;
            state.motor2Backward = false;
            state.motor2Stopped = false;
            Serial.println("State Change: Motor 2 Forward");
          }
        } else if(PS4.LStickY() < -100) {
          motor2Backward();
          if(!state.motor2Backward) {
            motorData.motor2 = -1;
            esp_err_t result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
            state.motor2Forward = false;
            state.motor2Backward = true;
            state.motor2Stopped = false;
            Serial.println("State Change: Motor 2 Backward");
          }
        } else {
          motor2Stop();
          if(!state.motor2Stopped) {
            motorData.motor2 = 0;
            esp_err_t result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
            while(result != ESP_OK) {
              result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
              Serial.println("Retrying Stop");
              delay(100);
            }
            state.motor2Forward = false;
            state.motor2Backward = false;
            state.motor2Stopped = true;
            Serial.println("State Change: Motor 2 Stop");
          }
        }
      }
      if (PS4.RStickY()) {
        if(PS4.RStickY() > 100) {
          motor1Forward();
          if(!state.motor1Forward) {
            motorData.motor1 = 1;
            esp_err_t result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
            state.motor1Forward = true;
            state.motor1Stopped = false;
            state.motor1Backward = false;
            Serial.println("State Change: Motor 1 Forward");
          }
        } else if(PS4.RStickY() < -100) {
          motor1Backward();
          if(!state.motor1Backward) {
            motorData.motor1 = -1;
            esp_err_t result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
            state.motor1Forward = false;
            state.motor1Stopped = false;
            state.motor1Backward = true;
            Serial.println("State Change: Motor 1 Backward");
          }
        } else {
          motor1Stop();
          if(!state.motor1Stopped) {
            motorData.motor1 = 0;
            esp_err_t result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
            Serial.printf("Result: %d\n", result);
            while(result != ESP_NOW_SEND_SUCCESS) {
              result = esp_now_send(0, (uint8_t *) &motorData, sizeof(motorData));
              Serial.println("Retrying Stop");
              delay(100);
            }
            state.motor1Forward = false;
            state.motor1Stopped = true;
            state.motor1Backward = false;
            Serial.println("State Change: Motor 1 Stop");
          }
        }
      }

    } else {
      Serial.println("Not connected");
      delay(1000);
    }
  }
}