#include "mbed.h"
#include "string"
#include "SPI_TFT_ILI9341.h"
#include "Arial28x28.h"
#include <chrono>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
SPI_TFT_ILI9341 TFT(PF_9, PF_8, PF_7, PC_2, PD_12, PD_13, "TFT"); // mosi, miso, sclk, cs, reset, dc
Timer timer;

// TFT related functions
void tft_init()
{
  TFT.claim(stdout); // send stdout to the TFT display
  TFT.set_orientation(1);
  TFT.background(Black);
  TFT.foreground(White);
  TFT.cls();
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 10);
  TFT.printf("Press enter to start.");
}

void tft_disp(const char *mode)
{
  TFT.claim(stdout); // send stdout to the TFT display
  TFT.set_orientation(1);
  TFT.background(Black);
  TFT.foreground(White);
  TFT.cls(); // 清除屏幕上的文本
  TFT.set_font((unsigned char *)Arial28x28);
  TFT.locate(15, 10);
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
volatile bool init_mode = true;

// x,y,z rotation values from gyro
int16_t final_x, final_y, final_z = 0;

// Variables that contain x,y,z rotation in radians
float datax;
float datay;
float dataz;

// Recorded sequence
const int max_sequence_length = 100;
float recorded_sequence[max_sequence_length][3]; // Store x, y, and z values
uint32_t recorded_timestamps[max_sequence_length];
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
  tft_init();
  gyro_init();
  timer.start();
  while (1)
  {
    ThisThread::sleep_for(chrono::milliseconds(100));
    printf("init_mode: %d\n", init_mode);
    printf("record_mode: %d\n", record_mode);
    printf("unlock_mode: %d\n", unlock_mode);
    ; // Add a wait to control the loop rate
    gyro_read();

    if (!init_mode)
    {
      if (record_mode)
      {
        tft_disp("Recording Mode");
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
}

void enter_key_handler()
{
  if (init_mode)
  {
    init_mode = false;
    tft_disp("Press button to start");
  }
  else if (!record_mode && !unlock_mode)
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
    led_indicator = 0;
    tft_disp("Unlock Mode Ended");
  }
  // Add a small delay to debounce the button press
  ThisThread::sleep_for(chrono::milliseconds(200));
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
  recorded_timestamps[sequence_length] = timer.read_ms();
}

bool compare_sequence()
{
  bool match = true;
  timer.reset();
  for (int i = 0; i < sequence_length; i++)
  {
    uint32_t time_to_wait = recorded_timestamps[i] - std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
    if (time_to_wait > 0)
    {
      ThisThread::sleep_for(chrono::milliseconds(time_to_wait));
    }
    gyro_read(); // Read the gyro data
    if (!(fabs(datax - recorded_sequence[i][0]) < tolerance &&
          fabs(datay - recorded_sequence[i][1]) < tolerance &&
          fabs(dataz - recorded_sequence[i][2]) < tolerance))
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
  spi.write(0x80 | 0x2A);          // Address of the OUT_Y_L register with read command (0x80 for read)
  final_y = spi.write(0x00);       // Read lower byte of OUT_Y
  final_y |= spi.write(0x00) << 8; // Read higher byte of OUT_Y and combine with lower byte
  spi.write(0x80 | 0x2C);          // Address of the OUT_Z_L register with read command (0x80 for read)
  final_z = spi.write(0x00);       // Read lower byte of OUT_Z
  final_z |= spi.write(0x00) << 8; // Read higher byte of OUT_Z and combine with lower byte
  cs = 1;                          // Set chip select to high (not selected)

  // Convert raw values to radians per second
  float dps_conversion = 8.75 / 1000;            // 8.75 mdps/LSB for 500 dps (degrees per second) full scale
  datax = final_x * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
  datay = final_y * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
  dataz = final_z * dps_conversion * M_PI / 180; // Convert degrees per second to radians per second
}
