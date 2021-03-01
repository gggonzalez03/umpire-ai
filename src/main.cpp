#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <Arduino.h>
#include <Wire.h>
#include "SPIFFS.h"

#include "I2Cdev.h"
#include "MPU6050.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define AZ_BUFFER_SIZE      100
#define MAX_BIN_DUMP_SIZE   1000000

#define LED_INDICATORS_ON   1
#define BUTTON_SELECTOR     1


/**************************************
 * UmpireAI State Enums
 **************************************/
enum {
  WAIT_SERVE_START = 0,
  WAIT_SERVE_END,
  WAIT_RECEIVER_HIT,
  WAIT_SERVER_HIT,
  RECEIVER_MISS,
  SERVER_MISS
};

enum {
  GAME_ONGOING = 0,
  BLACK_WINS,
  RED_WINS,
  DEUCE,
  BLACK_ADVANTAGE,
  RED_ADVANTAGE
};


/**************************************
 * UmpireAI State Variables
 **************************************/
MPU6050 mpu0(0x68);
MPU6050 mpu1(0x69);
int16_t a0[3], a1[3];
int az0_offset, az1_offset;
float az0_buffer[AZ_BUFFER_SIZE];
float az1_buffer[AZ_BUFFER_SIZE];
float az0, az1;

// To be used by identify_hit_task and binary_data_dump_task
float az0_shadow_buffer[AZ_BUFFER_SIZE];
float az1_shadow_buffer[AZ_BUFFER_SIZE];
uint8_t which_az_shadow_buffer;

float threshold = 1.5f;
uint8_t last_data_point_0 = AZ_BUFFER_SIZE, last_data_point_1 = AZ_BUFFER_SIZE;

uint8_t current_state = WAIT_SERVE_START, hit;
unsigned long hit_timestamp, current_timestamp;

uint8_t *server_score, *receiver_score, black_score, red_score, score_status;
uint8_t server; // server: 2 is none, server: 0 is black, 1 is red

File binary_dump_file;

const uint8_t BUTTON_PIN = 18;
const uint8_t LED_RED = 19;
const uint8_t LED_BLACK = 5;

/**************************************
 * UmpireAI Core Functions
 **************************************/
void game_state_update(uint8_t *current_state, unsigned long *hit_timestamp, unsigned long *current_timestamp, uint8_t *hit);
uint8_t handle_point_award(uint8_t* current_status, uint8_t* server_score, uint8_t* receiver_score);
uint8_t handle_serve_switch(uint8_t** server_score, uint8_t** receiver_score, uint8_t* server, uint8_t frequency);
uint8_t get_scoring_status(uint8_t black_score, uint8_t red_score);
void print_spiffs_contents(void);
void print_game_details(uint8_t black_score, uint8_t red_score, uint8_t current_server, uint8_t server_changed, uint8_t score_status);
void update_leds(uint8_t current_server);

/**************************************
 * ISRs
 **************************************/
void IRAM_ATTR isr();

/**************************************
 * UmpireAI Task Control Variables
 **************************************/
static SemaphoreHandle_t identify_hit_smphr;      // Signal identify_hit_task
static SemaphoreHandle_t tt_dynamics_smphr;       // Signal table_tennis_dynamics_task
static SemaphoreHandle_t dump_binary_data_smphr;  // Signal binary_data_dump_task
static SemaphoreHandle_t transmit_state_smphr;    // Signal bluetooth_transmit_state_task

/**************************************
 * UmpireAI Core Tasks
 **************************************/
void calibrate_imu_task(void *parameters);
void imu_task(void *parameters);
void identify_hit_task(void *parameters);
void table_tennis_dynamics_task(void *parameters);
void binary_data_dump_task(void *parameters);
void bluetooth_transmit_state_task(void *parameters);


/**************************************
 * BLE Variables
 **************************************/
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device Connected");
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }

    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

/**************************************
 * BLE Tasks
 **************************************/
void ble_task(void* parameters);


void setup() {
  Wire.setClock(400000);
  Wire.begin();
  Serial.begin(115200);

  print_spiffs_contents();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLACK, OUTPUT);

  attachInterrupt(BUTTON_PIN, isr, FALLING);

  identify_hit_smphr = xSemaphoreCreateBinary();
  tt_dynamics_smphr = xSemaphoreCreateBinary();
  dump_binary_data_smphr = xSemaphoreCreateBinary();
  transmit_state_smphr = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(calibrate_imu_task, "calibrate_imu_task", 4096, NULL, 5, NULL, 0);

  // Core 0 Tasks
  xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(identify_hit_task, "identify_hit_task", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(table_tennis_dynamics_task, "table_tennis_dynamics_task", 4096, NULL, 4, NULL, 0);

  // Core 1 Tasks
  xTaskCreatePinnedToCore(binary_data_dump_task, "binary_data_dump_task", 4096, NULL, 1, NULL, 1);
  // xTaskCreatePinnedToCore(ble_task, "ble_task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}

/**************************************
 * UmpireAI Core Tasks
 **************************************/
void calibrate_imu_task(void *parameters) {
  mpu0.initialize();
  mpu1.initialize();

  mpu0.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

  uint8_t i;
  
  for (i = 0; i < 100; i++) {
    mpu0.getAcceleration(&a0[0], &a0[1], &a0[2]);
    mpu1.getAcceleration(&a1[0], &a1[1], &a1[2]);

    az0_offset = (i * az0_offset + a0[2]) / (i + 1);
    az1_offset = (i * az1_offset + a1[2]) / (i + 1);
  }

  vTaskDelete(NULL);
}

void imu_task(void *parameters) {

  uint8_t buffer_position = 0;

  while(1) {
    mpu0.getAcceleration(&a0[0], &a0[1], &a0[2]);
    mpu1.getAcceleration(&a1[0], &a1[1], &a1[2]);
    
    az0 = (float)(a0[2] - az0_offset) / 2048.0f * 9.8f;
    az1 = (float)(a1[2] - az1_offset) / 2048.0f * 9.8f;

    az0_buffer[buffer_position] = az0;
    az1_buffer[buffer_position] = az1;

    // If a threshold is reached, record the index of the 100th point in the buffer
    if ((abs(az0) > threshold || abs(az1) > threshold) && (last_data_point_0 == AZ_BUFFER_SIZE && last_data_point_1 == AZ_BUFFER_SIZE)) {
      last_data_point_0 = (buffer_position + AZ_BUFFER_SIZE - 21) % AZ_BUFFER_SIZE; // -21 cos we want 20 data points before the trigger
      last_data_point_1 = (buffer_position + AZ_BUFFER_SIZE - 21) % AZ_BUFFER_SIZE;
      hit_timestamp = millis();
    }

    // Once the buffer is filled, identify which side of the table the ball hit
    if (buffer_position == last_data_point_0 || buffer_position == last_data_point_1) {

      xSemaphoreGive(identify_hit_smphr);

      last_data_point_0 = AZ_BUFFER_SIZE;
      last_data_point_1 = AZ_BUFFER_SIZE;
    }

    // Only try to decide scoring when an ongoing rally ended, aka the game is not currently waiting for a serve
    current_timestamp = millis();
    if (current_timestamp - hit_timestamp > 1500 && current_state != WAIT_SERVE_START) {
      xSemaphoreGive(tt_dynamics_smphr); 
    }

    buffer_position++;

    if (buffer_position == AZ_BUFFER_SIZE) {
      buffer_position = 0;
    }
  }
}

void identify_hit_task(void *parameters) {

  server = 0; // Select black as the first server

  while (1) {
    if (xSemaphoreTake(identify_hit_smphr, portMAX_DELAY)) {

      float az0_max = 0.0f, az1_max = 0.0f;
      float current_az0, current_az1;

      for (int i = 0; i < AZ_BUFFER_SIZE; i++) {

        current_az0 = az0_shadow_buffer[i] = az0_buffer[(i + last_data_point_0 + 1) % AZ_BUFFER_SIZE];
        current_az1 = az1_shadow_buffer[i] = az1_buffer[(i + last_data_point_1 + 1) % AZ_BUFFER_SIZE];

        // Serial.println(current_az0);
        // Serial.print(",");
        // Serial.println(current_az1);

        if (current_az0 > az0_max) {
          az0_max = current_az0;
        }
        if (current_az1 > az1_max) {
          az1_max = current_az1;
        }
      }

      if (az0_max > az1_max) {
        hit = server;
        which_az_shadow_buffer = 0;
      }
      else if (az0_max < az1_max) {
        hit = !server;
        which_az_shadow_buffer = 1;
      }
      xSemaphoreGive(dump_binary_data_smphr);
      xSemaphoreGive(tt_dynamics_smphr);
    }
  }
}

void table_tennis_dynamics_task(void *parameters) {

  server_score = &black_score;
  receiver_score = &red_score;

  while (1) {
    if (xSemaphoreTake(tt_dynamics_smphr, portMAX_DELAY)) {
      game_state_update(&current_state, &hit_timestamp, &current_timestamp, &hit);
      
      #ifdef LED_INDICATORS_ON
      if (current_state == 0) {
        update_leds(server);
      }
      else {
        update_leds(0xFF);
      }
      #endif
      
      if (handle_point_award(&current_state, server_score, receiver_score)) {
        score_status = get_scoring_status(black_score, red_score);
        uint8_t server_changed = 0;
        if (score_status == DEUCE || score_status == BLACK_ADVANTAGE || score_status == RED_ADVANTAGE) {
          server_changed = handle_serve_switch(&server_score, &receiver_score, &server, 1);
        }
        else {
          server_changed = handle_serve_switch(&server_score, &receiver_score, &server, 2);
        }

        print_game_details(black_score, red_score, server, server_changed, score_status);
      }
    }
  }
}

void binary_data_dump_task(void *parameters) {

  uint8_t* az_buffer = NULL;

  while (1) {
    if (xSemaphoreTake(dump_binary_data_smphr, portMAX_DELAY)) {

      if (!which_az_shadow_buffer) {
        az_buffer = (uint8_t*)az0_shadow_buffer;
      }
      else {
        az_buffer = (uint8_t*)az1_shadow_buffer;
      }

      binary_dump_file = SPIFFS.open("/data_dump.bin", "a+");
      
      if (binary_dump_file) {
        Serial.println(binary_dump_file.write(az_buffer, AZ_BUFFER_SIZE * sizeof(float)));

        if (binary_dump_file.size() >= MAX_BIN_DUMP_SIZE) {
          binary_dump_file.close();
          Serial.println("File closed.");
        }
        binary_dump_file.close();
      }
    }
  }
}

void bluetooth_transmit_state_task(void *parameters) {
  while (1) {

  }
}


/**************************************
 * UmpireAI Core Functions
 **************************************/

/**
 * This function determines the state of the game.
 * 
 * @param current_state the current state of the game. This will get updated after this function call.
 * @param hit_timestamp the timestamp in ms when the ball hit the table.
 * @param current_timestamp the current timestamp.
 * @param hit which side of the table was hit. 2 for no hit, 0 for server side table hit, and 1 for receiver side table hit.
 * @return void.
 * */
void game_state_update(uint8_t *current_state, unsigned long *hit_timestamp, unsigned long *current_timestamp, uint8_t *hit) {
  switch (*current_state)
  {
  case WAIT_SERVE_START:
    if (*hit == 0) {
      *current_state = WAIT_SERVE_END; // Set current_state to the next state for the next iteration
    }
    break;
  case WAIT_SERVE_END:
    if (*hit == 1) {
      *current_state = WAIT_RECEIVER_HIT;
    }
    if ((*current_timestamp - *hit_timestamp) > 1500) {
      *current_state = WAIT_SERVE_START;
    }
    break;
  case WAIT_RECEIVER_HIT:
    if (*hit == 0) {
      *current_state = WAIT_SERVER_HIT;
    }
    if ((*current_timestamp - *hit_timestamp) > 1500) {
      *current_state = RECEIVER_MISS;
    }
    break;
  case WAIT_SERVER_HIT:
    if (*hit == 1) {
      *current_state = WAIT_RECEIVER_HIT;
    }
    if ((*current_timestamp - *hit_timestamp) > 1500) {
      *current_state = SERVER_MISS;
    }
    break;
  case RECEIVER_MISS:
    *current_state = WAIT_SERVE_START;
    break;
  case SERVER_MISS:
    *current_state = WAIT_SERVE_START;
    break;

  default:
    *current_state = WAIT_SERVE_START;
    break;
  }

  *hit = 2; // This is required to clear the hit
}

/**
 * Award the point to the correct side
 * @param current_status is the current status of the game.
 * @param server_score pointer to the uint8_t score variable of the current server.
 * @param receiver_score pointer to the uint8_t score variable of the current receiver.
 * @return 1 if point is awarded, 0 if not.
 * */
uint8_t handle_point_award(uint8_t* current_status, uint8_t* server_score, uint8_t* receiver_score) {
  if (*current_status == RECEIVER_MISS) {
    (*server_score)++;
    return 1;
  }
  else if (*current_status == SERVER_MISS) {
    (*receiver_score)++;
    return 1;
  }

  return 0;
}

/**
 * Change the server if required
 * @param server_score pointer to the uint8_t score variable of the current server.
 * @param receiver_score pointer to the uint8_t score variable of the current receiver.
 * @param frequency is how many serves before switch
 * @return 1 if switch occurred, 0 if not.
 * */
uint8_t handle_serve_switch(uint8_t** server_score, uint8_t** receiver_score, uint8_t* server, uint8_t frequency) {
  uint8_t* temp = *server_score;
  if (((**server_score) + (**receiver_score)) % frequency == 0) {
    *server_score = *receiver_score;
    *receiver_score = temp;
    *server = !(*server);
    return 1;
  }

  return 0;
}

/**
 * Announce the status of the scoring, whether there's a winner, a deuce,
 * and who has the advantage.
 * @param black_score black score.
 * @param red_score red score.
 * @return scoring status.
 * */
uint8_t get_scoring_status(uint8_t black_score, uint8_t red_score) {

  if (black_score == 10 && red_score == 10) {
    return DEUCE;
  }

  uint8_t higher_score = black_score >= red_score ? black_score : red_score;

  if (higher_score < 11) {
    return GAME_ONGOING;
  }

  int8_t score_diff = black_score - red_score;

  if (abs(score_diff) >= 2) {
    // If score_diff is negative, it means red_score is higher, and is the winner's score
    if (score_diff < 0) {
      return RED_WINS;
    }
    else {
      return BLACK_WINS;
    }
  }
  else if (abs(score_diff) == 1) {
    // If score_diff is negative, it means red_score is higher
    if (score_diff < 0) {
      return RED_ADVANTAGE;
    }
    else {
      return BLACK_ADVANTAGE;
    }
  }
  else if (score_diff == 0) {
    return DEUCE;
  }

  return GAME_ONGOING;
}

void print_spiffs_contents(void) {
  // This will allocate 1.5MB of the flash to SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
 
  File root = SPIFFS.open("/");
 
  File file = root.openNextFile();
 
  while(file){
 
      Serial.print("FILE: ");
      Serial.println(file.name());
      Serial.println(file.size());
 
      file = root.openNextFile();
  }
}

/**
 * Print the details of the game such as who's serving, who has advantage in a deuce, and score
 * @param black_score black score.
 * @param red_score red score.
 * @param current_server the side that's currently serving.
 * @param server_changed is wheter the server just changed.
 * @param score_status is whether the game is ongoing, there is a deuce, or the winner is decided.
 * */
void print_game_details(uint8_t black_score, uint8_t red_score, uint8_t current_server, uint8_t server_changed, uint8_t score_status) {

  switch (score_status)
  {
  case GAME_ONGOING:
    Serial.println("Status: GAME_ONGOING");
    break;
  case BLACK_WINS:
    Serial.println("Status: BLACK_WINS");
    break;
  case RED_WINS:
    Serial.println("Status: RED_WINS");
    break;
  case DEUCE:
    Serial.println("Status: DEUCE");
    break;
  case BLACK_ADVANTAGE:
    Serial.println("Status: BLACK_ADVANTAGE");
    break;
  case RED_ADVANTAGE:
    Serial.println("Status: RED_ADVANTAGE");
    break;
  
  default:
    Serial.println("Status: GAME_ONGOING");
    break;
  }

  Serial.printf("Black: %d, Red: %d\n", black_score, red_score);

  if (score_status != BLACK_WINS && score_status != RED_WINS) {
    if (server_changed) {
      Serial.println("Server changed");
    }
    if (current_server == 0) {
      Serial.println("Black is serving");
    }
    else {
      Serial.println("Red is serving");
    }
  }

  Serial.print("\n");
}

void update_leds(uint8_t current_server) {
  switch (current_server) {
  case 0:
    // black is serving
    digitalWrite(LED_BLACK, HIGH);
    digitalWrite(LED_RED, LOW);
    break;
  case 1:
    // red is serving
    digitalWrite(LED_BLACK, LOW);
    digitalWrite(LED_RED, HIGH);
    break;
  
  default:
    digitalWrite(LED_BLACK, LOW);
    digitalWrite(LED_RED, LOW);
    break;
  }
}
/**************************************
 * ISRs
 **************************************/
void IRAM_ATTR isr() {
  Serial.println("Hello from ISR\n");
}


/**************************************
 * BLE Tasks
 **************************************/
void ble_task(void* parameters) {
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  uint32_t data = 0;
  while (1) {
    data = 0;
    data |= (black_score) | (red_score << 8) | (server << 16) | (score_status << 24);
    // notify changed value
    if (deviceConnected) {
        pCharacteristic->setValue((uint8_t*)&data, 4);
        pCharacteristic->notify();
        delay(1000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
  }
}