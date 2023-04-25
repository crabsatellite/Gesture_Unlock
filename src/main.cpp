#include "stdio.h"
#include "mbed.h"
#include "Arial28x28.h"
#include "SPI_TFT_ILI9341.h"
#include "string"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// the TFT is connected to SPI pin 11-14
SPI_TFT_ILI9341 TFT(PF_9, PF_8, PF_7, PC_2, PD_12, PD_13, "TFT"); // mosi, miso, sclk, cs, reset, dc

// TFT related functions
void tft_init()
{
  TFT.claim(stdout); // send stdout to the TFT display
  TFT.set_orientation(1);
  TFT.background(Black);
  TFT.foreground(White);
  TFT.cls();
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 130);
  TFT.printf("Mode: ");
}

void tft_disp(const char *mode, float dist = 0, float cal = 0, int steps = 0)
{
  TFT.foreground(White);
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 130);
  TFT.printf("Mode: %s", mode);
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

// x,y,z rotation values from gyro
int16_t final_x, final_y, final_z = 0;

// Variables that contain x,y,z rotation in radians
float datax;
float datay;
float dataz;

// Recorded sequence
const int max_sequence_length = 100;
float recorded_sequence[max_sequence_length][3]; // Store x, y, and z values
int sequence_length = 0;

// Tolerance for gesture comparison
const float tolerance = 0.05;

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

  gyro_init();
  tft_init();
  while (1)
  {
    gyro_read();

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
    tft_disp("Record Mode Ended");
    unlock_mode = true;
    tft_disp("Unlock Mode");
  }
  else if (unlock_mode)
  {
    unlock_mode = false;
    tft_disp("Unlock Mode Ended");
  }
}

void record_sequence()
{
  if (sequence_length < max_sequence_length)
  {
    recorded_sequence[sequence_length][0] = datax;
    recorded_sequence[sequence_length][1] = datay;
    recorded_sequence[sequence_length][2] = dataz;
    sequence_length++;
  }
  else
  {
    record_mode = false;
    led_indicator = 0;
    tft_disp("Record Mode Ended (Max Length)");
  }
}

bool compare_sequence()
{
  bool match = true;
  for (int i = 0; i < sequence_length; i++)
  {
    if (abs(datax - recorded_sequence[i][0]) > tolerance ||
        abs(datay - recorded_sequence[i][1]) > tolerance ||
        abs(dataz - recorded_sequence[i][2]) > tolerance)
    {
      match = false;
      break;
    }
  }
  return match;
}

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
}

void gyro_read()
{
  cs = 0;                          // Set chip select to low (selected)
  spi.write(0x80 | 0x28);          // Address of the OUT_X_L register with read command (0x80 for read)
  final_x = spi.write(0x00);       // Read lower byte of OUT_X
  final_x |= spi.write(0x00) << 8; // Read higher byte of OUT_X and combine with lower byte
  final_y = spi.write(0x00);       // Read lower byte of OUT_Y
  final_y |= spi.write(0x00) << 8; // Read higher byte of OUT_Y and combine with lower byte
  final_z = spi.write(0x00);       // Read lower byte of OUT_Z
  final_z |= spi.write(0x00) << 8; // Read higher byte of OUT_Z and combine with lower byte
  cs = 1;                          // Set chip select to high (not selected)

  // Convert raw values to radians per second
  float dps_conversion = 8.75 / 1000;            // 8.75 mdps/LSB for 500 dps (degrees per second) full scale
  datax = final_x * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
  datay = final_y * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
  dataz = final_z * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
}