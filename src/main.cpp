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

#define LED_INDICATORS_ON   0
#define BUTTON_SELECTOR     0


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
  BLACK_WINS_GAME,
  RED_WINS_GAME,
  BLACK_WINS_MATCH,
  RED_WINS_MATCH,
  DEUCE,
  BLACK_ADVANTAGE,
  RED_ADVANTAGE
};

/**************************************
 * UmpireAI Bluetooth Control Enums
 **************************************/
enum {
  NONE = 0,
  RESTART_MATCH,
  RESTART_GAME,
  INCREMENT_BLACK_SCORE,
  DECREMENT_BLACK_SCORE,
  INCREMENT_RED_SCORE,
  DECREMENT_RED_SCORE,
  START_DATA_TRANSFER_OVER_BLE,
  COLLECT_HIT_DATA,
  COLLECT_NON_HIT_DATA,
  STOP_DATA_COLLECTION,
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
float threshold = 1.5f;
uint8_t last_data_point_0 = AZ_BUFFER_SIZE, last_data_point_1 = AZ_BUFFER_SIZE;

// To be used by identify_hit_task and binary_data_dump_task
uint8_t data_dump_enabled;
float az0_shadow_buffer[AZ_BUFFER_SIZE];
float az1_shadow_buffer[AZ_BUFFER_SIZE];
uint8_t which_az_shadow_buffer;
float data_label = 0.0f;

uint8_t sets_count;
uint8_t current_state = WAIT_SERVE_START, hit;
unsigned long hit_timestamp, current_timestamp;

uint8_t best_of_match = 3;
uint8_t game_winner_declared, match_winner_declared;
uint8_t black_match_score, red_match_score;
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
uint8_t handle_match_point_award(uint8_t* current_status, uint8_t* black_match_score, uint8_t* red_match_score);
uint8_t handle_serve_switch(uint8_t** server_score, uint8_t** receiver_score, uint8_t* server, uint8_t frequency);
uint8_t get_scoring_status(uint8_t black_score, uint8_t red_score);
uint8_t handle_ble_command(uint8_t command);
void handle_service_indicator(uint8_t *previous_state, uint8_t *current_state);
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
#define SERVICE_UUID        "4a220a7a-8094-435b-8b3e-b19682b41381"
#define CHARACTERISTIC_UUID_RX "638b6661-6196-4efc-82f4-e90a59e6e8a3"
#define CHARACTERISTIC_UUID_TX  "30ab87d1-2e52-4874-a519-888c6fc54bcc"

uint8_t is_ble_client_connected = 0;
BLECharacteristic *tx_characteristic;
uint8_t tx_buffer[8];

/**************************************
 * BLE Classes
 **************************************/
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer *ble_server) {
    // Only allow two simultaneous connections
    if (++is_ble_client_connected < 2) {
      BLEDevice::startAdvertising();
    }
  }

  void onDisconnect(BLEServer *ble_server) {
    --is_ble_client_connected;
  }
};

class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string rx_value = characteristic->getValue();
    const char* rx_decoded = rx_value.c_str();

    if (rx_value.length() > 0) {
      Serial.printf("%d\n", rx_decoded[0]);
      handle_ble_command((uint8_t)rx_decoded[0]);
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

  #ifdef LED_INDICATORS_ON
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLACK, OUTPUT);
  #endif

  #ifdef BUTTON_SELECTOR
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PIN, isr, FALLING);
  #endif

  data_dump_enabled = 0;
  sets_count = 5;

  identify_hit_smphr = xSemaphoreCreateBinary();
  tt_dynamics_smphr = xSemaphoreCreateBinary();
  dump_binary_data_smphr = xSemaphoreCreateBinary();
  transmit_state_smphr = xSemaphoreCreateCounting(5, 0);

  xTaskCreatePinnedToCore(calibrate_imu_task, "calibrate_imu_task", 4096, NULL, 5, NULL, 0);

  // Core 0 Tasks
  xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(identify_hit_task, "identify_hit_task", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(table_tennis_dynamics_task, "table_tennis_dynamics_task", 4096, NULL, 4, NULL, 0);

  // Core 1 Tasks
  xTaskCreatePinnedToCore(binary_data_dump_task, "binary_data_dump_task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ble_task, "ble_task", 4096, NULL, 1, NULL, 1);
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

      if (data_dump_enabled) {
        xSemaphoreGive(dump_binary_data_smphr);
      }

      xSemaphoreGive(tt_dynamics_smphr);
    }
  }
}

void table_tennis_dynamics_task(void *parameters) {

  server_score = &black_score;
  receiver_score = &red_score;

  uint8_t previous_state = 0;

  while (1) {
    if (xSemaphoreTake(tt_dynamics_smphr, portMAX_DELAY)) {
      previous_state = current_state;
      game_state_update(&current_state, &hit_timestamp, &current_timestamp, &hit);
      
      #ifdef LED_INDICATORS_ON
      if (current_state == 0) {
        update_leds(server);
      }
      else {
        update_leds(0xFF);
      }
      #endif
      
      handle_service_indicator(&previous_state, &current_state);
      // Serial.print(previous_state);
      // Serial.print(", ");
      // Serial.print(current_state);
      // Serial.print(", ");
      // Serial.print(server);
      // Serial.print(", ");
      // Serial.println(hit);
      
      if (handle_point_award(&current_state, server_score, receiver_score)) {
        score_status = get_scoring_status(black_score, red_score);
        uint8_t server_changed = 0;
        if (score_status == DEUCE || score_status == BLACK_ADVANTAGE || score_status == RED_ADVANTAGE) {
          server_changed = handle_serve_switch(&server_score, &receiver_score, &server, 1);
        }
        else if (score_status == BLACK_WINS_GAME || score_status == RED_WINS_GAME) {
          handle_match_point_award(&score_status, &black_match_score, &red_match_score);
          game_winner_declared = 1;
        }
        else if (score_status == BLACK_WINS_MATCH || score_status == RED_WINS_MATCH) {
          match_winner_declared = 1;
        }
        else {
          server_changed = handle_serve_switch(&server_score, &receiver_score, &server, 2);
        }

        xSemaphoreGive(transmit_state_smphr);

        print_game_details(black_score, red_score, server, server_changed, score_status);
        Serial.printf("%d, %d\n", black_match_score, red_match_score);
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
        binary_dump_file.write(az_buffer, AZ_BUFFER_SIZE * sizeof(float));
        binary_dump_file.write((uint8_t*)(&data_label), 1 * sizeof(float));

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

  if (game_winner_declared) {
    return 0;
  }

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
 * Award the maatch point to the correct player
 * @param current_status is the current status of the game.
 * @param black_match_score pointer to the current black match score
 * @param red_match_score pointer to the current red match score
 * @return 1 if point is awarded, 0 if not.
 **/ 
uint8_t handle_match_point_award(uint8_t* current_status, uint8_t* black_match_score, uint8_t* red_match_score) {

  if (game_winner_declared || match_winner_declared) {
    return 0;
  }

  if (*current_status == BLACK_WINS_GAME) {
    (*black_match_score)++;
  }
  else if (*current_status == RED_WINS_GAME) {
    (*red_match_score)++;
  }

  if (*black_match_score >= best_of_match) {
    (*current_status) = BLACK_WINS_MATCH;
  }
  if (*red_match_score >= best_of_match) {
    (*current_status) = RED_WINS_MATCH;
  }

  return 1;
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

  uint8_t status = GAME_ONGOING;

  if (black_score == 10 && red_score == 10) {
    status = DEUCE;
  }

  uint8_t higher_score = black_score >= red_score ? black_score : red_score;

  if (higher_score < 11) {
    return GAME_ONGOING;
  }

  int8_t score_diff = black_score - red_score;

  if (abs(score_diff) >= 2) {
    // If score_diff is negative, it means red_score is higher, and is the winner's score
    if (score_diff < 0) {
      status = RED_WINS_GAME;
    }
    else {
      status = BLACK_WINS_GAME;
    }
  }
  else if (abs(score_diff) == 1) {
    // If score_diff is negative, it means red_score is higher
    if (score_diff < 0) {
      status = RED_ADVANTAGE;
    }
    else {
      status = BLACK_ADVANTAGE;
    }
  }
  else if (score_diff == 0) {
    status = DEUCE;
  }

  return status;
}

uint8_t handle_ble_command(uint8_t command) {

  switch (command) {
  case NONE:
    break;
  case RESTART_MATCH:
    // Reset scores and server
    black_score = red_score = server = 0;
    server_score = &black_score;
    receiver_score = &red_score;
    game_winner_declared = match_winner_declared = 0;

    // Reset match scores
    black_match_score = red_match_score = 0;

    score_status = GAME_ONGOING;

    // Give semaphore to ble task to send new game state
    xSemaphoreGive(transmit_state_smphr);
    break;
  case RESTART_GAME:
    // Reset scores and server
    black_score = red_score = server = 0;
    server_score = &black_score;
    receiver_score = &red_score;
    game_winner_declared = 0;

    score_status = GAME_ONGOING;

    // Give semaphore to ble task to send new game state
    xSemaphoreGive(transmit_state_smphr);
    break;
  case INCREMENT_BLACK_SCORE:
    black_score = ++black_score;
    xSemaphoreGive(transmit_state_smphr);
    break;
  case DECREMENT_BLACK_SCORE:
    black_score = black_score ? --black_score : 0;
    xSemaphoreGive(transmit_state_smphr);
    break;
  case INCREMENT_RED_SCORE:
    red_score = ++red_score;
    xSemaphoreGive(transmit_state_smphr);
    break;
  case DECREMENT_RED_SCORE:
    red_score = red_score ? --red_score : 0;
    xSemaphoreGive(transmit_state_smphr);
    break;
  case START_DATA_TRANSFER_OVER_BLE:
    break;
  case COLLECT_HIT_DATA:
    data_dump_enabled = 1;
    data_label = 1;
    break;
  case COLLECT_NON_HIT_DATA:
    data_dump_enabled = 2;
    data_label = 0;
    break;
  case STOP_DATA_COLLECTION:
    data_dump_enabled = 0;
    break;

  default:
    break;
  }

  return command;
}

/**
 * This functions updates indicator controls for the ble task. The goal is to turn off indicator
 * when the serve has been done, and then turn it on when it's time to serve again.
 * @param previous_state Previous state of the game
 * @param current_state Current state of the game (after calling game_state_update)
 * */
void handle_service_indicator(uint8_t *previous_state, uint8_t *current_state) {
  if (*previous_state == WAIT_SERVE_START && *current_state == WAIT_SERVE_END) {
    // *server = 2;
    xSemaphoreGive(transmit_state_smphr);
  }
  else if (*previous_state != WAIT_SERVE_START && *current_state == WAIT_SERVE_START) {
    // *server = 2;
    xSemaphoreGive(transmit_state_smphr);
  }
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
  case BLACK_WINS_GAME:
    Serial.println("Status: BLACK_WINS_GAME");
    break;
  case RED_WINS_GAME:
    Serial.println("Status: RED_WINS_GAME");
    break;
  case BLACK_WINS_MATCH:
    Serial.println("Status: BLACK_WINS_MATCH");
    break;
  case RED_WINS_MATCH:
    Serial.println("Status: RED_WINS_MATCH");
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

  if (score_status != BLACK_WINS_GAME && score_status != RED_WINS_GAME) {
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
  BLEDevice::init("UmpireAI");
  BLEServer *ble_server = BLEDevice::createServer();

  ble_server->setCallbacks(new ServerCallbacks());
  BLEService *service = ble_server->createService(SERVICE_UUID);

  // Create service for data transmit
  tx_characteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
    );
 
  tx_characteristic->addDescriptor(new BLE2902());

  // Create service for data receive
  BLECharacteristic *rx_characteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
    );
  rx_characteristic->setCallbacks(new CharacteristicCallbacks());

  service->start();
  ble_server->getAdvertising()->start();

  while (1) {
    if (xSemaphoreTake(transmit_state_smphr, portMAX_DELAY) && is_ble_client_connected) {
      tx_buffer[0] = score_status;
      tx_buffer[1] = black_score;
      tx_buffer[2] = red_score;
      tx_buffer[3] = (server << 0) | (current_state << 4);
      tx_buffer[4] = black_match_score;
      tx_buffer[5] = red_match_score;
      
      tx_characteristic->setValue(tx_buffer, 6);
      tx_characteristic->notify();
    }
  }
}