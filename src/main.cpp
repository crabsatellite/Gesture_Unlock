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
SPI_TFT_ILI9341 TFT(PF_9, PF_8, PF_7, PC_2, PD_12, PD_13, "TFT"); // mosi, miso, sclk, cs, reset, dc
Timer timer;

// Recorded sequence
const int max_sequence_length = 100;
float recorded_sequence[max_sequence_length][3]; // Store x, y, and z values
uint32_t recorded_timestamps[max_sequence_length];
int sequence_length = 0;          // Number of recorded gestures
const int window_size = 50;       // Adjust the window size, increase it will make it more robust to noise, but will also make it less responsive
const float match_threshold = 50; // Adjust the match threshold as needed, decrease it will make it more sensitive, but will also make it less robust to noise
// Tolerance for gesture comparison (calculating the difference between two angles), increase it will make it easier to unlock, but will also make it less secure
const float tolerance = 10.0;
std::vector<std::vector<float>> buffer(window_size, std::vector<float>(3, 0)); // Buffer to store the last 7 values of x, y, and z
// TFT related functions
void tft_init()
{
  TFT.claim(stdout); // send stdout to the TFT display

  TFT.background(Black); // set background to black
  TFT.foreground(White); // set chars to white
  TFT.cls();             // clear the screen

  TFT.background(Black);
  TFT.cls();

  TFT.cls();
  TFT.rect(0, 0, 400, 400, LightGrey);

  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 10);
  TFT.printf("ECE-GY 6483");

  TFT.foreground(Red);
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 50);
  TFT.printf("Guesture");

  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 90);
  TFT.printf("Unlock");

  TFT.foreground(Blue);
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 140);
  TFT.printf("Alex");
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 180);
  TFT.printf("Alhad");
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 220);
  TFT.printf("Aman, S");
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 260);
  TFT.printf("Aman, V");

  wait_us(2000000);

  TFT.cls();
  TFT.rect(0, 0, 400, 400, LightGrey);
  TFT.foreground(White); // set chars to white
}

void tft_disp(const char *mode)
{
  TFT.claim(stdout); // send stdout to the TFT display
  TFT.set_orientation(0);
  TFT.background(Black);
  TFT.foreground(White);
  TFT.cls(); // 清除屏幕上的文本
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 10);
  TFT.printf("%s", mode);
}

void tft_disp_debugger(float value, int posY)
{
  TFT.set_orientation(0);
  TFT.background(Black);
  TFT.foreground(White);
  TFT.cls();
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, posY);
  TFT.printf("%.2f", value);
}

// Gyro spi pins
SPI spi(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);

// LED indicator
DigitalOut led_indicator(LED1);

// "Enter key" and "Record" button
InterruptIn enter_key(USER_BUTTON);
volatile bool record_mode = false;
volatile bool unlock_mode = false;
volatile bool button_pressed = false;
// x,y,z rotation values from gyro
int16_t final_x, final_y, final_z = 0;

// Variables that contain x,y,z rotation in radians
float datax;
float datay;
float dataz;

// Function prototypes
void gyro_init(); // Function to initialize L3GD20 gyro
void gyro_read(); // Function to read L3GD20 gyro data
void enter_key_handler();
void record_sequence();
bool compare_sequence();

int main()
{
  // Attach interrupt handlers for "Enter key"
  enter_key.fall(&enter_key_handler);
  tft_init();
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
    printf("x: %f, y: %f, z: %f\n", datax, datay, dataz);

    if (record_mode)
    {
      record_sequence();
    }
    else if (unlock_mode)
    {
      bool match = compare_sequence();
      if (match)
      {
        led_indicator = 1;
        unlock_mode = false; // Exit unlock mode when the gesture is matched
        tft_disp("Unlocked");
      }
      else
      {
        led_indicator = 0;
      }
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
  if (!record_mode && !unlock_mode)
  {
    record_mode = true;
    sequence_length = 0;
    led_indicator = 1;
    tft_disp("Recording Mode");
  }
  else if (record_mode)
  {

    record_mode = false;
    led_indicator = 0;
    unlock_mode = true;
    tft_disp("Unlock Mode");
  }
  button_pressed = false;
}

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

void record_sequence()
{
  if (sequence_length < max_sequence_length)
  {
    buffer.erase(buffer.begin());
    buffer.push_back({datax, datay, dataz});

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

    recorded_sequence[sequence_length][0] = avg_x;
    recorded_sequence[sequence_length][1] = avg_y;
    recorded_sequence[sequence_length][2] = avg_z;
    sequence_length++;
    // debug for avg values (String avg_， int pos)
    // printf("avg_x: %.2f, avg_y: %.2f, avg_z: %.2f\n", avg_x, avg_y, avg_z);
    wait_us(200000); // Add a 200 ms delay between each recorded data point
  }
  else
  {
    record_mode = false;
    led_indicator = 0;
    tft_disp("Record Mode Ended (Max Length)");
  }
  if (sequence_length > 0)
  {
    recorded_timestamps[sequence_length - 1] = timer.read_ms() - recorded_timestamps[sequence_length - 1];
  }
  if (sequence_length < max_sequence_length)
  {
    recorded_timestamps[sequence_length] = timer.read_ms();
  }
}

bool compare_sequence()
{
  std::vector<std::vector<float>> current_sequence;
  uint32_t start_time = timer.read_ms();
  uint32_t current_time = start_time;
  int i = 0;

  while (current_time - start_time < recorded_timestamps[sequence_length - 1])
  {
    current_time = timer.read_ms();
    if (current_time - start_time >= recorded_timestamps[i])
    {
      gyro_read(); // Read the gyro data
      current_sequence.push_back({datax, datay, dataz});
      ++i;
    }
  }

  // Convert recorded_sequence to the same format as current_sequence
  std::vector<std::vector<float>> recorded_sequence_vector;
  for (int i = 0; i < sequence_length; i++)
  {
    recorded_sequence_vector.push_back({recorded_sequence[i][0], recorded_sequence[i][1], recorded_sequence[i][2]});
  }

  float dtw_dist = dtw_distance(recorded_sequence_vector, current_sequence);

  if (dtw_dist <= match_threshold)
  {
    led_indicator = 1;
    return true;
  }
  else
  {
    led_indicator = 0;
    return false;
  }
}

/*
  Function to initialize L3GD20 gyro
*/
void gyro_init()
{
  cs = 1;                 // Set chip select to high (not selected)
  spi.format(8, 3);       // Set SPI to 8-bit mode, with mode 3 (CPOL=1, CPHA=1)
  spi.frequency(1000000); // Set SPI frequency to 1MHz

  // Write the control register 1 (CTRL_REG1) to enable the gyro
  cs = 0;          // Set chip select to low (selected)
  spi.write(0x20); // Address of the CTRL_REG1 with write command
  spi.write(0x0F); // Data to enable gyro (0b00001111)
  cs = 1;          // Set chip select to high (not selected)

  cs = 0;          // Set chip select to low (selected)
  spi.write(0x23); // Address of the CTRL_REG4 with write command
  spi.write(0x20); // Data to set full scale to 2000 dps (0b00100000)
  cs = 1;          // Set chip select to high (not selected)
}

void gyro_read()
{
  cs = 0;                          // Set chip select to low (selected)
  spi.write(0x80 | 0x28);          // Address of the OUT_X_L register with read command (0x80 for read)
  final_x = spi.write(0x00);       // Read lower byte of OUT_X
  final_x |= spi.write(0x00) << 8; // Read higher byte of OUT_X and combine with lower byte
  spi.write(0x80 | 0x2A);          // Address of the OUT_Y_L register with read command (0x80 for read)
  final_y = spi.write(0x00);       // Read lower byte of OUT_Y
  final_y |= spi.write(0x00) << 8; // Read higher byte of OUT_Y and combine with lower byte
  spi.write(0x80 | 0x2C);          // Address of the OUT_Z_L register with read command (0x80 for read)
  final_z = spi.write(0x00);       // Read lower byte of OUT_Z
  final_z |= spi.write(0x00) << 8; // Read higher byte of OUT_Z and combine with lower byte
  cs = 1;                          // Set chip select to high (not selected)

  // Convert raw values to radians per second
  float dps_conversion = 70.0 / 1000;            // 70 mdps/LSB for 2000 dps (degrees per second) full scale
  datax = final_x * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
  datay = final_y * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
  dataz = final_z * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
}
