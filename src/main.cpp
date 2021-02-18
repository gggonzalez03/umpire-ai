#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#define AZ_BUFFER_SIZE 100


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
int16_t a0[3], a1[3];
int az0_offset, az1_offset;
uint8_t buffer_position, last_data_point_0 = AZ_BUFFER_SIZE, last_data_point_1 = AZ_BUFFER_SIZE;
float threshold = 1.5f;
int az0_hit, az1_hit;
uint8_t current_state = WAIT_SERVE_START, hit, black_score, red_score;
uint8_t *server_score, *receiver_score; // Assuming that black is the first server
uint8_t server; // server: 2 is none, server: 0 is black, 1 is red
unsigned long az0_hit_timestamp, az1_hit_timestamp, hit_timestamp, current_timestamp;

float az0_buffer[AZ_BUFFER_SIZE];
float az1_buffer[AZ_BUFFER_SIZE];

float az0, az1;

MPU6050 mpu0(0x68);
MPU6050 mpu1(0x69);

/**************************************
 * UmpireAI Core Functions
 **************************************/
void game_state_update(uint8_t *current_state, unsigned long *hit_timestamp, unsigned long *current_timestamp, uint8_t *hit);
uint8_t handle_point_award(uint8_t* current_status, uint8_t* server_score, uint8_t* receiver_score);
uint8_t handle_serve_switch(uint8_t** server_score, uint8_t** receiver_score, uint8_t* server, uint8_t frequency);
uint8_t get_scoring_status(uint8_t black_score, uint8_t red_score);
void print_game_details(uint8_t black_score, uint8_t red_score, uint8_t current_server, uint8_t server_changed, uint8_t score_status);


/**************************************
 * UmpireAI Core Tasks
 **************************************/
void umpire_ai_task(void *parameters);

void setup() {
  Wire.setClock(400000);
  Wire.begin();

  Serial.begin(115200);

  xTaskCreate(&umpire_ai_task, "umpire_ai_task", 8192, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void umpire_ai_task(void *parameters) {
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

  server_score = &black_score;
  receiver_score = &red_score;
  server = 0; // Select black as the first server

  while(1) {
    mpu0.getAcceleration(&a0[0], &a0[1], &a0[2]);
    mpu1.getAcceleration(&a1[0], &a1[1], &a1[2]);
    
    az0 = (float)(a0[2] - az0_offset) / 2048.0f * 9.8f;
    az1 = (float)(a1[2] - az1_offset) / 2048.0f * 9.8f;

    az0_buffer[buffer_position] = az0;
    az1_buffer[buffer_position] = az1;

    // Serial.println(az0);

    if ((abs(az0) > threshold || abs(az1) > threshold) && (last_data_point_0 == AZ_BUFFER_SIZE && last_data_point_1 == AZ_BUFFER_SIZE)) {
      last_data_point_0 = (buffer_position + AZ_BUFFER_SIZE - 21) % AZ_BUFFER_SIZE; // -21 cos we want 20 data points before the trigger
      last_data_point_1 = (buffer_position + AZ_BUFFER_SIZE - 21) % AZ_BUFFER_SIZE;
      hit_timestamp = millis();
    }

    if (buffer_position == last_data_point_0 || buffer_position == last_data_point_1) {

      int i;
      float az0_max = 0.0f, az1_max = 0.0f;
      for (i = 0; i < AZ_BUFFER_SIZE; i++) {
        // Serial.println(az0_buffer[(i + last_data_point_0 + 1) % AZ_BUFFER_SIZE]);
        // Serial.print(",");
        // Serial.println(az1_buffer[(i + last_data_point_1 + 1) % AZ_BUFFER_SIZE]);
        if (az0_buffer[(i + last_data_point_0 + 1) % AZ_BUFFER_SIZE] > az0_max) {
          az0_max = az0_buffer[(i + last_data_point_0 + 1) % AZ_BUFFER_SIZE];
        }
        if (az1_buffer[(i + last_data_point_1 + 1) % AZ_BUFFER_SIZE] > az1_max) {
          az1_max = az1_buffer[(i + last_data_point_1 + 1) % AZ_BUFFER_SIZE];
        }
      }

      if (az0_max > az1_max) {
        az0_hit_timestamp = millis();
        hit = server;
        az0_hit++;
      }
      else if (az0_max < az1_max) {
        az1_hit_timestamp = millis();
        hit = !server;
        az1_hit++;
      }

      last_data_point_0 = AZ_BUFFER_SIZE;
      last_data_point_1 = AZ_BUFFER_SIZE;

      // Serial.print(az0_hit);
      // Serial.print(", \t");
      // Serial.print(az1_hit);
      // Serial.print(", \t");
      // Serial.print(az1_hit_timestamp - az0_hit_timestamp);
      // Serial.print(" ms");
      // Serial.print(", \t");
      // Serial.print(az0_hit_timestamp - az1_hit_timestamp);
      // Serial.println(" ms");
    }

    current_timestamp = millis();
    game_state_update(&current_state, &hit_timestamp, &current_timestamp, &hit);
    
    if (handle_point_award(&current_state, server_score, receiver_score)) {
      uint8_t score_status = get_scoring_status(black_score, red_score);
      uint8_t server_changed = 0;
      if (score_status == DEUCE || score_status == BLACK_ADVANTAGE || score_status == RED_ADVANTAGE) {
        server_changed = handle_serve_switch(&server_score, &receiver_score, &server, 1);
      }
      else {
        server_changed = handle_serve_switch(&server_score, &receiver_score, &server, 2);
      }

      print_game_details(black_score, red_score, server, server_changed, score_status);
    }

    // Serial.println(current_state);

    buffer_position++;

    if (buffer_position == AZ_BUFFER_SIZE) {
      buffer_position = 0;
    }
  }
}

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