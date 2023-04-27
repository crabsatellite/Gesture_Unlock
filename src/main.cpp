#include "mbed.h"
#include "string"
#include "SPI_TFT_ILI9341.h"
#include "Arial28x28.h"
#include <chrono>
#include <vector>
#include <limits>
#include "BufferedSerial.h"
#include <cstdio>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
Timer timer;

// Recorded sequence
const int max_sequence_length = 100;
float recorded_sequence[max_sequence_length][3]; // Store x, y, and z values
float attempt_sequence[max_sequence_length][3];  // Store x, y, and z values
uint32_t recorded_timestamps[max_sequence_length];
uint32_t attempt_timestamps[max_sequence_length];
int sequence_length = 0;                                                       // Number of recorded gestures
int attempt_sequence_length = 0;                                               // Number of recorded gestures
const int window_size = 100;                                                   // Adjust the window size, increase it will make it more robust to noise, but will also make it less responsive
const float tolerance = 50;                                                    // Adjust the match tolerance as needed, if the DTW distance is less than this tolerance, then the gesture is matched. Decrease it to make it more strict
float dtw_dist = std::numeric_limits<float>::max();                            // Initialize the DTW distance to a very large number
std::vector<std::vector<float>> buffer(window_size, std::vector<float>(3, 0)); // Buffer to store the last 7 values of x, y, and z
std::vector<std::vector<float>> recorded_sequence_vector;
std::vector<std::vector<float>> attempt_sequence_vector;

// Gyro spi pins
SPI spi(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);

// LED indicator
DigitalOut led_indicator(LED1);

// "Enter key" and "Record" button
InterruptIn enter_key(USER_BUTTON);
bool record_mode = false;
volatile bool locked_mode = false;
volatile bool button_pressed = false;
bool attempt_locked_mode = false;
volatile bool unlock_success = false;
// x,y,z rotation values from gyro
int16_t raw_x, raw_y, raw_z = 0;

// Variables that contain x,y,z rotation in radians
float datax, datay, dataz = 0;

// final_x, final_y, final_z are the final x,y,z values after filtering
float final_x, final_y, final_z = 0;

// Function prototypes
void gyro_init(); // Function to initialize L3GD20 gyro
void gyro_read(); // Function to read L3GD20 gyro data
void enter_key_handler();
void record_sequence(float sequence[][3], int &sequence_length, bool &mode);
bool compare_sequence();
int main()
{
  // Attach interrupt handlers for "Enter key"
  enter_key.fall(&enter_key_handler);
  // tft_init();
  gyro_init();
  timer.start();

  auto last_button_press_time = std::chrono::steady_clock::now();

  while (1)
  {
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_press = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_button_press_time).count();

    if (button_pressed && time_since_last_press > 100)
    {
      enter_key_handler();
      last_button_press_time = now;
    }

    gyro_read();

    if (record_mode)
    {
      record_sequence(recorded_sequence, sequence_length, record_mode);
    }

    if (attempt_locked_mode)
    {
      record_sequence(attempt_sequence, attempt_sequence_length, attempt_locked_mode);
    }
  }
}

void enter_key_handler()
{
  if (!button_pressed)
  {
    button_pressed = true;
    return;
  }
  if (!record_mode && !locked_mode && !attempt_locked_mode)
  {
    record_mode = true;
    sequence_length = 0;
    led_indicator = 1;
    // tft_disp("Recording...");
    printf("Recording...\n");
  }
  else if (record_mode)
  {
    record_mode = false;
    led_indicator = 0;
    locked_mode = true;
    // tft_disp("Press Enter\n to Unlock");
    printf("Press Enter to Unlock\n");
  }
  else if (locked_mode)
  {
    locked_mode = false;
    attempt_locked_mode = true;
    // tft_disp("Attempting...");
    printf("Attempting...\n");
  }
  else if (attempt_locked_mode)
  {
    bool match = compare_sequence(); // Call the compare_sequence function here
    if (match)
    {
      // tft_disp("Successful!");
      printf("Successful!\n");
      attempt_locked_mode = false;
      unlock_success = true;
      led_indicator = 1;
    }
    else
    {
      // tft_disp("Failed");
      printf("Failed\n");
      locked_mode = true;
      attempt_locked_mode = false;
      unlock_success = false;
      led_indicator = 0;
    }
    // clear the attempt sequence
    attempt_sequence_length = 0;
    attempt_sequence_vector.clear();
    for (int i = 0; i < max_sequence_length; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        attempt_sequence[i][j] = 0.0;
      }
    }
  }
  button_pressed = false;
}

float dtw_distance(const std::vector<std::vector<float>> seq1, const std::vector<std::vector<float>> seq2)
{
  int len1 = seq1.size();
  int len2 = seq2.size();

  std::vector<std::vector<float>> dtw(len1 + 1, std::vector<float>(len2 + 1, std::numeric_limits<float>::infinity()));

  dtw[0][0] = 0;

  for (int i = 1; i <= len1; ++i)
  {
    for (int j = 1; j <= len2; ++j)
    {
      float cost = 0;
      for (int k = 0; k < 3; ++k)
      {
        cost += std::abs(seq1[i - 1][k] - seq2[j - 1][k]);
      }
      dtw[i][j] = cost + std::min({dtw[i - 1][j], dtw[i][j - 1], dtw[i - 1][j - 1]});
    }
  }

  return dtw[len1][len2];
}
void record_sequence(float sequence[][3], int &sequence_length, bool &mode)
{
  if (sequence_length < max_sequence_length)
  {
    buffer.erase(buffer.begin());
    buffer.push_back({final_x, final_y, final_z});

    float avg_x = 0;
    float avg_y = 0;
    float avg_z = 0;

    for (int i = 0; i < window_size; ++i)
    {
      avg_x += buffer[i][0];
      avg_y += buffer[i][1];
      avg_z += buffer[i][2];
    }

    avg_x /= window_size;
    avg_y /= window_size;
    avg_z /= window_size;

    sequence[sequence_length][0] = avg_x;
    sequence[sequence_length][1] = avg_y;
    sequence[sequence_length][2] = avg_z;
    sequence_length++;
    wait_us(200000); // Add a 200 ms delay between each recorded data point
  }
  else
  {
    mode = false;
    led_indicator = 0;
    // tft_disp("Record Mode Ended (Max Length)");
  }
}

vector<vector<float>> get_recorded_sequence()
{
  vector<vector<float>> recorded_sequence_vector;
  for (int i = 0; i < sequence_length; ++i)
  {
    recorded_sequence_vector.push_back({recorded_sequence[i][0], recorded_sequence[i][1], recorded_sequence[i][2]});
  }
  return recorded_sequence_vector;
}

vector<vector<float>> get_attempt_sequence()
{
  // print attempt sequence
  // for (int i = 0; i < sequence_length; ++i)
  // {
  //   printf("%f, %f, %f\n", attempt_sequence[i][0], attempt_sequence[i][1], attempt_sequence[i][2]);
  // }
  vector<vector<float>> attempt_sequence_vector;
  for (int i = 0; i < attempt_sequence_length; ++i)
  {
    attempt_sequence_vector.push_back({attempt_sequence[i][0], attempt_sequence[i][1], attempt_sequence[i][2]});
  }
  return attempt_sequence_vector;
}

bool compare_sequence()
{
  vector<vector<float>> answer_sequence_vector = get_recorded_sequence();
  vector<vector<float>> attempt_sequence_vector = get_attempt_sequence();
  printf("Recorded sequence:\n");
  for (const auto &vector : answer_sequence_vector)
  {
    printf("%f, %f, %f\n", vector[0], vector[1], vector[2]);
  }

  printf("attempt sequence:\n");
  for (const auto &vector : attempt_sequence_vector)
  {
    printf("%f, %f, %f\n", vector[0], vector[1], vector[2]);
  }

  float dtw_dist = dtw_distance(answer_sequence_vector, attempt_sequence_vector);
  printf("DTW Distance: %f\n", dtw_dist);
  if (dtw_dist <= tolerance)
  {
    led_indicator = 1;
    unlock_success = true;
    return true;
  }
  else
  {
    led_indicator = 0;
    unlock_success = false;
    return false;
  }
}
/*
  Function to initialize L3GD20 gyro
*/
void gyro_init()
{
  // Chip must be deselected
  cs = 1;
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8, 3);
  spi.frequency(1000000);

  ////////////Who am I????
  cs = 0;
  // wait_us(200);
  // Send 0x8f, the command to read the WHOAMI register
  spi.write(0x8F);
  // Send a dummy byte to receive the contents of the WHOAMI register
  int whoami = spi.write(0x00);

  ////////////Control Register 1
  // Read Control1 Register
  spi.write(0xA0);
  int ctrl1 = spi.write(0x00);
  // printf("Control1 = 0x%X\n", ctrl1);

  cs = 1;
  cs = 0;
  spi.write(0x20);
  spi.write(0x0F);
  cs = 1;

  cs = 0;
  // Read Control1 Register
  ctrl1 = spi.write(0x00);
  // printf("control1 = 0x%X\n", ctrl1);
  cs = 1;

  ////////////Control Register 4
  // Read Control4 Register
  cs = 0;
  spi.write(0x24);
  spi.write(0x12);
  cs = 1;
  cs = 0;
  // Read Control4 Register
  spi.write(0xA4);

  // Send a dummy byte to receive the contents of the Control 4 register
  int ctrl4 = spi.write(0x00);
  // printf("control4 = 0x%X\n", ctrl4);
  cs = 1;

  ////////////Control Register 3
  cs = 0;
  // Read Control3 Register
  spi.write(0xA3);

  // Send a dummy byte to receive the contents of the Control 3 register
  int ctrl3 = spi.write(0x00);
  // printf("control3 = 0x%X\n", ctrl3);
  cs = 1;

  cs = 0;
  spi.write(0x23);
  spi.write(0x20);
  cs = 1;
  // Send a dummy byte to receive the contents of the Control 2 register
  ctrl3 = spi.write(0x00);
  // printf("control3 = 0x%X\n", ctrl3);
  cs = 1;
}

/*
  Function to read L3GD20 gyro data and compute the distance
*/
void gyro_read()
{
  cs = 0;
  spi.write(0xE8);
  int OUT_X_L = spi.write(0x00);
  int OUT_X_H = spi.write(0x00);
  int OUT_Y_L = spi.write(0x00);
  int OUT_Y_H = spi.write(0x00);
  int OUT_Z_L = spi.write(0x00);
  int OUT_Z_H = spi.write(0x00);
  cs = 1;

  int16_t raw_x = (OUT_X_H << 8) | (OUT_X_L);
  int16_t raw_y = (OUT_Y_H << 8) | (OUT_Y_L);
  int16_t raw_z = (OUT_Z_H << 8) | (OUT_Z_L);

  // rps = raw_x * dps * (degrees_to_radians)
  // 0.00875 * 0.0174533 = 0.000152716
  datax = raw_x + 700;
  datay = raw_y + 800;
  dataz = raw_z + 700;
  final_x = datax;
  final_y = datay;
  final_z = dataz;
  // printf("x = %f, y = %f, z = %f\n", datax, datay, dataz);

  // printf("x = %f, y = %f, z = %f\n", final_x, final_y, final_z);
}