#include "mbed.h"
#include "string"
#include "SPI_TFT_ILI9341.h"
#include "Arial28x28.h"
#include <chrono>
#include <vector>
#include <limits>
#include <cstdio>
#include <cstdint>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// max_sequence_length is the maximum number of gestures that can be recorded
const int max_sequence_length = 300;
// recorded_sequence and attempt_sequence are the arrays that store the recorded and attempt gestures
float recorded_sequence[max_sequence_length][3]; // Store x, y, and z values
float attempt_sequence[max_sequence_length][3];  // Store x, y, and z values
int sequence_length = 0;
int attempt_sequence_length = 0;
// window_size is the number of values to be stored in the buffer, used for DTW
// Adjust the window size, increase it will make it more robust to noise, but will also make it less responsive
const int window_size = 5;
// Adjust the match tolerance as needed, if the DTW distance is less than this tolerance, then the gesture is matched. Decrease it to make it more strict
// Choose a tolerance according to the amplitude of the gesture, if the amplitude is large, then the tolerance should be large as well
// consider the value now a placeholder! More testing is needed to find a good value
// Probably need to ask for the clarification of the "gesture"
const float tolerance = 1500;
// These are the variables used for DTW
std::vector<std::vector<float>> buffer(window_size, std::vector<float>(3, 0)); // Buffer to store the last 7 values of x, y, and z
std::vector<std::vector<float>> recorded_sequence_vector;
std::vector<std::vector<float>> attempt_sequence_vector;

// These are the variables used for the gyroscope
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
// raw_x, raw_y, raw_z are the raw x,y,z values directly from the gyroscope
int16_t raw_x, raw_y, raw_z = 0;
// datax, datay, dataz are the x,y,z values after calibration
float datax, datay, dataz = 0;

// final_x, final_y, final_z are the final x,y,z values after filtering
float final_x, final_y, final_z = 0;

// calibration values
float data_offset_x, data_offset_y, data_offset_z = 0;
// These are the variables used for the gyroscope
constexpr int WHO_AM_I = 0x8F;
constexpr int CTRL_REG1 = 0x20;
constexpr int CTRL_REG1_CONFIG = 0x0F;
constexpr int CTRL_REG3 = 0x23;
constexpr int CTRL_REG3_CONFIG = 0x20;
constexpr int CTRL_REG4 = 0x24;
constexpr int CTRL_REG4_CONFIG = 0x12;
// Function prototypes
void gyro_init();
void gyro_calibrate();
void gyro_read();
void enter_key_handler();
void record_sequence(float sequence[][3], int &sequence_length, bool &mode);
bool compare_sequence();

int main()
{
  // Initialize the arrays for recorded_sequence and attempt_sequence
  for (int i = 0; i < max_sequence_length; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      attempt_sequence[i][j] = 0.0;
    }
  }
  attempt_sequence_length = 0;
  attempt_sequence_vector.clear();
  // Attach interrupt handlers for "Enter key"
  enter_key.fall(&enter_key_handler);
  // tft_init();

  // Initialize the gyroscope
  gyro_calibrate();
  gyro_init();

  // Initialize the buffer
  auto last_button_press_time = std::chrono::steady_clock::now();

  while (1)
  {
    // Read the gyroscope
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_press = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_button_press_time).count();
    gyro_read();
    // Read the button
    if (button_pressed && time_since_last_press > 100)
    {
      enter_key_handler();
      last_button_press_time = now;
    }
    std::fill(buffer.begin(), buffer.end(), std::vector<float>(3, 0.0));
    if (record_mode)
    {
      record_sequence(recorded_sequence, sequence_length, record_mode);
    }
    std::fill(buffer.begin(), buffer.end(), std::vector<float>(3, 0.0));
    if (attempt_locked_mode)
    {
      record_sequence(attempt_sequence, attempt_sequence_length, attempt_locked_mode);
    }
  }
}

/*
  Handles the "Enter key" interrupt
*/
void handle_record_mode()
{
  record_mode = false;
  led_indicator = 0;
  locked_mode = true;
  printf("Press Enter to Unlock\n");
}

void handle_locked_mode()
{
  locked_mode = false;
  attempt_locked_mode = true;
  attempt_sequence_length = 0;
  printf("Attempting...\n");
}

void handle_attempt_locked_mode()
{
  // compare the recorded sequence and the attempt sequence
  bool match = compare_sequence();
  if (match)
  {
    printf("Successful!\n");
    attempt_locked_mode = false;
    unlock_success = true;
    led_indicator = 1;
  }
  else
  {
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
    printf("Recording...\n");
  }
  else if (record_mode)
  {
    handle_record_mode();
  }
  else if (locked_mode)
  {
    handle_locked_mode();
  }
  else if (attempt_locked_mode)
  {
    handle_attempt_locked_mode();
  }
  button_pressed = false;
}

/*
  Function to calculate the DTW distance between two sequences
*/
float dtw_distance(const std::vector<std::vector<float>> &seq1, const std::vector<std::vector<float>> &seq2)
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

/*
  Function to record the sequence
*/
void record_sequence(float sequence[][3], int &sequence_length, bool &mode)
{
  if (sequence_length < max_sequence_length)
  {
    buffer.erase(buffer.begin());
    buffer.push_back({final_x, final_y, final_z});
    // printf("x: %f, y: %f, z: %f\n", final_x, final_y, final_z);

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
    // printf("x: %f, y: %f, z: %f\n", avg_x, avg_y, avg_z);
    float threshold = 40.0;
    if ((avg_x < threshold && avg_x > -threshold) && (avg_y < threshold && avg_y > -threshold) && (avg_z < threshold && avg_z > -threshold))
    {
      return;
    }

    sequence[sequence_length][0] = avg_x;
    sequence[sequence_length][1] = avg_y;
    sequence[sequence_length][2] = avg_z;
    sequence_length++;
    wait_us(50000); // Add a delay between each recorded data point
  }
  else
  {
    mode = false;
    led_indicator = 0;
    // tft_disp("Record Mode Ended (Max Length)");
    printf("Record Mode Ended (Max Length)\n");
  }
}

std::vector<std::vector<float>> get_recorded_sequence()
{
  std::vector<std::vector<float>> recorded_sequence_vector;
  for (int i = 0; i < sequence_length; ++i)
  {
    recorded_sequence_vector.push_back({recorded_sequence[i][0], recorded_sequence[i][1], recorded_sequence[i][2]});
  }
  return recorded_sequence_vector;
}

std::vector<std::vector<float>> get_attempt_sequence()
{
  std::vector<std::vector<float>> attempt_sequence_vector;
  for (int i = 0; i < attempt_sequence_length; ++i)
  {
    attempt_sequence_vector.push_back({attempt_sequence[i][0], attempt_sequence[i][1], attempt_sequence[i][2]});
  }
  return attempt_sequence_vector;
}

/*
  Function to compare the recorded sequence and the attempt sequence
*/
bool compare_sequence()
{
  std::vector<std::vector<float>> answer_sequence_vector = get_recorded_sequence();
  std::vector<std::vector<float>> attempt_sequence_vector = get_attempt_sequence();
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
void write_spi_register(int address, int value)
{
  cs = 0;
  spi.write(address);
  spi.write(value);
  cs = 1;
}

void gyro_init()
{
  // Chip must be deselected
  cs = 1;
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8, 3);
  spi.frequency(1000000);

  // Read WHO_AM_I register
  cs = 0;
  spi.write(WHO_AM_I);
  int whoami = spi.write(0x00);
  cs = 1;
  // printf("WHOAMI = 0x%X\n", whoami);

  // Write to CTRL_REG1
  write_spi_register(CTRL_REG1, CTRL_REG1_CONFIG);

  // Write to CTRL_REG4
  write_spi_register(CTRL_REG4, CTRL_REG4_CONFIG);

  // Write to CTRL_REG3
  write_spi_register(CTRL_REG3, CTRL_REG3_CONFIG);
}
/*
  Function to calibrate the gyro
*/
void gyro_calibrate()
{
  int calibration_samples = 100;
  float offsetX = 0, offsetY = 0, offsetZ = 0;

  for (int i = 0; i < calibration_samples; i++)
  {
    gyro_read(); // read raw data
    offsetX += datax;
    offsetY += datay;
    offsetZ += dataz;
    wait_us(10000); // wait for 10 ms between samples
  }

  offsetX /= calibration_samples;
  offsetY /= calibration_samples;
  offsetZ /= calibration_samples;

  // Store the offsets
  data_offset_x = offsetX;
  data_offset_y = offsetY;
  data_offset_z = offsetZ;
}

#include <deque>

// Moving average filter window size
const int moving_average_window = 5;

std::deque<float> x_buffer(moving_average_window);
std::deque<float> y_buffer(moving_average_window);
std::deque<float> z_buffer(moving_average_window);

/*
  Moving average filter function, used to filter the gyro data
*/
float moving_average(std::deque<float> &buffer, float new_value)
{
  buffer.pop_front();
  buffer.push_back(new_value);

  float sum = 0;
  for (float value : buffer)
  {
    sum += value;
  }

  return sum / buffer.size();
}

/*
  Function to read L3GD20 gyro data and compute the distance
*/

struct GyroData
{
  int16_t x;
  int16_t y;
  int16_t z;
};

GyroData read_raw_gyro_data()
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

  GyroData raw_data;
  raw_data.x = (OUT_X_H << 8) | (OUT_X_L);
  raw_data.y = (OUT_Y_H << 8) | (OUT_Y_L);
  raw_data.z = (OUT_Z_H << 8) | (OUT_Z_L);

  return raw_data;
}

GyroData subtract_offset(const GyroData &raw_data, int16_t data_offset_x, int16_t data_offset_y, int16_t data_offset_z)
{
  GyroData data;
  data.x = raw_data.x - data_offset_x;
  data.y = raw_data.y - data_offset_y;
  data.z = raw_data.z - data_offset_z;
  return data;
}

bool is_below_threshold(const GyroData &data, float threshold)
{
  return (data.x < threshold && data.x > -threshold) &&
         (data.y < threshold && data.y > -threshold) &&
         (data.z < threshold && data.z > -threshold);
}

void gyro_read()
{
  GyroData raw_data = read_raw_gyro_data();
  GyroData data = subtract_offset(raw_data, data_offset_x, data_offset_y, data_offset_z);

  float threshold = 150.0;
  if (is_below_threshold(data, threshold))
  {
    return;
  }

  // Apply moving average filter
  float filtered_x = moving_average(x_buffer, data.x);
  float filtered_y = moving_average(y_buffer, data.y);
  float filtered_z = moving_average(z_buffer, data.z);

  GyroData filtered_data = {filtered_x, filtered_y, filtered_z};
  if (is_below_threshold(filtered_data, threshold))
  {
    return;
  }

  final_x = filtered_x;
  final_y = filtered_y;
  final_z = filtered_z;
  // printf("x = %f, y = %f, z = %f\n", final_x, final_y, final_z);
}
