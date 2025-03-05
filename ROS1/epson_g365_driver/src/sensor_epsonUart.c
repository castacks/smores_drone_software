//==============================================================================
//
//  sensor_epsonUart.c - Epson IMU sensor protocol UART specific code
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
//  SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
//  SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
//
//==============================================================================
#include "sensor_epsonCommon.h"
#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"

// These are declared by the main() application for UART IF
extern const char *IMUSERIAL;   // COM port device name
extern int comPort;             // COM port handle

unsigned char rxBuffer[256];    // COM port receive buffer

// UART Interface Timing
// TWRITERATE/TREADRATE = 200us min @ 460800 BAUD, 1 command = 3 bytes = 3 * 22us = 66us
// TSTALL = 200us - 66us = 134us
#define EPSON_STALL                 134     // Microseconds

// UART Byte Markers
#ifdef V340
const unsigned char UART_HEADER = 0x20;  // Placed at the start of all UART cycles
#else
const unsigned char UART_HEADER = 0x80;  // Placed at the start of all UART cycles
#endif
const unsigned char UART_DELIMITER = 0x0D;  // Placed at the end of all UART cycles


/*****************************************************************************
** Function name:       sensorDataByteLength
** Description:         Determines the sensor burst read packet data length
**                      based on the parameters in the EpsonOptions struct.
** Parameters:          options - struct describing IMU configuration.
** Return value:        data byte length
*****************************************************************************/
unsigned int sensorDataByteLength(struct EpsonOptions options)
{

  unsigned int length = 0;

#ifdef V340
  // V340 has fixed packet format with optional 16-bit count value
  length = 18;

#else
  // 16 bit ND_EA FLAG
  if(options.flag_out)
    length += 2;

  // 16 or 32 bit Temperature Output
  if(options.temp_out)
  {
    if(options.temp_bit)
      length += 4;
    else
      length += 2;
  }

  // 16 or 32 bit Gyro X, Y, Z Output
  if(options.gyro_out)
  {
    if(options.gyro_bit)
      length += 12;
    else
      length += 6;
  }

  // 16 or 32 bit Accl X, Y, Z Output
  if(options.accel_out)
  {
    if(options.accel_bit)
      length += 12;
    else
      length += 6;
  }

  // 16 or 32 bit Delta Angle X, Y, Z Output
  if(options.gyro_delta_out)
  {
    if(options.gyro_delta_bit)
      length += 12;
    else
      length += 6;
  }

  // 16 or 32 bit Delta Velocity X, Y, Z Output
  if(options.accel_delta_out)
  {
    if(options.accel_delta_bit)
      length += 12;
    else
      length += 6;
  }

#if defined G365PDC0 || defined G365PDF0 || defined G325PDF0
  // 16 or 32 bit Attitude X, Y, Z Output
  // NOTE: Only supported by G365PDC0/G365PDF0/G325PDF0
  if(options.atti_out)
  {
    if(options.atti_bit)
      length += 12;
    else
      length += 6;
  }
#endif

  // 16 bit GPIO status output
  if(options.gpio_out)
    length += 2;
  if(options.checksum_out)
    length += 2;
#endif

  // 16 bit Count output
  if(options.count_out)
    length += 2;

  // For Start and End byte
  length += 2;

  return length;
}


/*****************************************************************************
** Function name:       sensorDataReadyOptions
** Description:         For UART interface check if comport recv buffer
**                      contains a burst of data based on expected byte length
**                      from sensorDataByteLength()
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int sensorDataReadyOptions(struct EpsonOptions options)
{

  seDelayMicroSecs(100);
  int count = numBytesReadComPort(comPort);

  if(count >= sensorDataByteLength(options))
    return SUCCESS;
  return FAIL;
}


/*****************************************************************************
** Function name:       registerWriteByte
** Description:         Write Byte to Register = Set WIN_ID, Write Data
**                      to Register
**                      NOTE: G350/V340 does not have WINDOW_ID function
**                            winNumber input parameter is ignored
** Parameters:          Window Number, Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByte(unsigned char winNumber, unsigned char regAddr, unsigned char regByte, unsigned int verbose)
{
  unsigned char txData[3];

#if !(defined G350 || defined V340)
  txData[0] = ADDR_WIN_CTRL|0x80;  //msb is 1b for register writes
  txData[1] = winNumber;
  txData[2] = UART_DELIMITER;
  writeComPort(comPort, txData, 3);
  EpsonStall();
#endif

  txData[0] = regAddr | 0x80; // msb is 1b for register writes
  txData[1] = regByte;
  txData[2] = UART_DELIMITER;
  writeComPort(comPort, txData, 3);
  EpsonStall();

  if (verbose)
  {
    printf("\r\nREG[0x%02X(W%01X)] < 0x%02X\t", regAddr, winNumber, regByte);
  }
}

/*****************************************************************************
** Function name:       registerRead16
** Description:         Read 16-bit from Register
**                      NOTE: G350/V340 does not have WINDOW_ID function
**                            winNumber input parameter is ignored
** Parameters:          Window Number, Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16(unsigned char winNumber, unsigned char regAddr, unsigned int verbose)
{
  unsigned char response[4] = {0};
  int size;
  unsigned char txData[3];

#if !(defined G350 || defined V340)
  txData[0] = ADDR_WIN_CTRL|0x80;
  txData[1] = winNumber;
  txData[2] = UART_DELIMITER;
  writeComPort(comPort, txData, 3);
  EpsonStall();
#endif

  txData[0] = regAddr & 0x7E; // msb is 0b for register reads & address must be even
  txData[1] = 0x00;
  txData[2] = UART_DELIMITER;
  writeComPort(comPort, txData, 3);

  EpsonStall();

  // Attempt to read 4 bytes from serial port
  // Validation check: Should be atleast 4 bytes, First byte should be Register Address,
  //                   Last byte should be delimiter
  size = readComPort(comPort, &response[0], 4);
  if ((size<4) || (response[0] != txData[0]) || (response[3] != UART_DELIMITER))
    printf("Returned less data or unexpected data from previous command.\n");
    EpsonStall();

  if (verbose)
  {
    printf("REG[0x%02X(W%01X)] > 0x%02X%02X\t", regAddr, winNumber, response[1], response[2]);
  }
  return (unsigned short)response[1]<<8|(unsigned short)response[2];
}

/*****************************************************************************
** Function name:       sensorRecoverFromGarbageInit
** Description:         Recover from IMU in weird state
** Parameters:          None
** Return value:        None
*****************************************************************************/

void sensorRecoverFromGarbageInit(void)
{
  unsigned char response[4] = {0};
  int size;
  unsigned char txData[2];
  txData[0]=0x0D;
  do{
    writeComPort(comPort, txData, 1);
    seDelayMicroSecs(5000);
    size = numBytesReadComPort(comPort);
    if(size>=4)
      size = readComPort(comPort, &response[0], 4);
    printf("\r\n...Checking if response is good.");
   } while(size<4);
  printf("\r\n...Response was good continuing.");
  //printf("\r\n...Software Reset begin.");
  //registerWriteByte(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_SOFTRESET, debug);
  //seDelayMS(EPSON_POWER_ON_DELAY);
  //printf("\r\n...Software Reset complete.");

}


static unsigned char data[256];
#define START 0
#define DATA 1
#define END 2
static int state = START;
static int data_count = 0;
static int scale_factors_initialized = 0;
static double dv_scale_factors[16];
static double da_scale_factors[16];

/*****************************************************************************
** Function name:       populateEpsonData
** Description:         Loads Delta Angle/Velocity scale factors the one time
**                      Retrieves burst data buffer and converts/parses into struct
**                      based on configuration.
** Parameters:          options - struct describing IMU configuration.
**                      epson_data - struct that is filled with converted data.
** Return value:        none
** Notes:
******************************************************************************/
void populateEpsonData(struct EpsonOptions options, struct EpsonData* epson_data)
{

  // If not done so, store Delta Angle/Velocity scale factors
  if(!scale_factors_initialized){
    da_scale_factors[0] = EPSON_DA_SF0;
    da_scale_factors[1] = EPSON_DA_SF1;
    da_scale_factors[2] = EPSON_DA_SF2;
    da_scale_factors[3] = EPSON_DA_SF3;
    da_scale_factors[4] = EPSON_DA_SF4;
    da_scale_factors[5] = EPSON_DA_SF5;
    da_scale_factors[6] = EPSON_DA_SF6;
    da_scale_factors[7] = EPSON_DA_SF7;
    da_scale_factors[8] = EPSON_DA_SF8;
    da_scale_factors[9] = EPSON_DA_SF9;
    da_scale_factors[10] = EPSON_DA_SF10;
    da_scale_factors[11] = EPSON_DA_SF11;
    da_scale_factors[12] = EPSON_DA_SF12;
    da_scale_factors[13] = EPSON_DA_SF13;
    da_scale_factors[14] = EPSON_DA_SF14;
    da_scale_factors[15] = EPSON_DA_SF15;

    dv_scale_factors[0] = EPSON_DV_SF0;
    dv_scale_factors[1] = EPSON_DV_SF1;
    dv_scale_factors[2] = EPSON_DV_SF2;
    dv_scale_factors[3] = EPSON_DV_SF3;
    dv_scale_factors[4] = EPSON_DV_SF4;
    dv_scale_factors[5] = EPSON_DV_SF5;
    dv_scale_factors[6] = EPSON_DV_SF6;
    dv_scale_factors[7] = EPSON_DV_SF7;
    dv_scale_factors[8] = EPSON_DV_SF8;
    dv_scale_factors[9] = EPSON_DV_SF9;
    dv_scale_factors[10] = EPSON_DV_SF10;
    dv_scale_factors[11] = EPSON_DV_SF11;
    dv_scale_factors[12] = EPSON_DV_SF12;
    dv_scale_factors[13] = EPSON_DV_SF13;
    dv_scale_factors[14] = EPSON_DV_SF14;
    dv_scale_factors[15] = EPSON_DV_SF15;
    scale_factors_initialized = 1;
  }

  // stores the sensor data array index when parsing out data fields
  int idx = 0;

#ifdef V340
  // Fixed packet format except for enabling/disabling count_out
  unsigned short ndflags = (data[idx] << 8) + data[idx + 1];
  epson_data->ndflags = ndflags;
  idx += 2;

  short temp = (data[idx] << 8) + data[idx+1];
  epson_data->temperature = (temp - 2634)*EPSON_TEMP_SF + 25;
  idx += 2;
  
  short gyro_x = (data[idx] << 8) + data[idx + 1];
  short gyro_y = (data[idx + 2] << 8) + data[idx + 3];
  short gyro_z = (data[idx + 4] << 8) + data[idx + 5];
  epson_data->gyro_x = EPSON_GYRO_SF*3.14159/180.0 * gyro_x;
  epson_data->gyro_y = EPSON_GYRO_SF*3.14159/180.0 * gyro_y;
  epson_data->gyro_z = EPSON_GYRO_SF*3.14159/180.0 * gyro_z;
  idx += 6;

  short accel_x = (data[idx] << 8) + (data[idx] + 1);
  short accel_y = (data[idx + 2] << 8) + (data[idx + 3]);
  short accel_z = (data[idx + 4] << 8) + (data[idx + 5]);
  epson_data->accel_x = (EPSON_ACCL_SF)*9.80665/1000.0 * accel_x;
  epson_data->accel_y = (EPSON_ACCL_SF)*9.80665/1000.0 * accel_y;
  epson_data->accel_z = (EPSON_ACCL_SF)*9.80665/1000.0 * accel_z;
  idx += 6;
  unsigned short gpio = (data[idx] << 8) + data[idx + 1];
  idx += 2;
  epson_data->gpio = gpio;

#else
  // parsing of data fields applying conversion factor if applicable
  if(options.flag_out)
  {
    unsigned short ndflags = (data[idx] << 8) + data[idx + 1];
    epson_data->ndflags = ndflags;
    idx += 2;
#ifdef DEBUG
    printf("ndflag: %04x\t", epson_data->ndflags);
#endif
  }

  if(options.temp_out)
  {
    if(options.temp_bit)
    {
      int temp = (data[idx] << 8*3) + (data[idx + 1] << 8*2) + (data[idx + 2] << 8) + data[idx + 3];
      epson_data->temperature = (temp - 172621824)*EPSON_TEMP_SF/65536 + 25;
      idx += 4;
    }
    else
    {
      short temp = (data[idx] << 8) + data[idx + 1];
      epson_data->temperature = (temp - 2634)*EPSON_TEMP_SF + 25;
      idx += 2;
    }
#ifdef DEBUG
    printf("tempC: %8.3f\t", epson_data->temperature);
#endif
  }

  if(options.gyro_out)
  {
    if(options.gyro_bit)
    {
      int gyro_x = (data[idx] << 8*3) + (data[idx + 1] << 8*2) + (data[idx + 2] << 8) + data[idx + 3];
      int gyro_y = (data[idx + 4] << 8*3) + (data[idx + 5] << 8*2) + (data[idx + 6] << 8) + data[idx + 7];
      int gyro_z = (data[idx + 8] << 8*3) + (data[idx + 9] << 8*2) + (data[idx + 10] << 8) + data[idx + 11];
      epson_data->gyro_x = (EPSON_GYRO_SF/65536)*3.14159/180.0 * gyro_x;
      epson_data->gyro_y = (EPSON_GYRO_SF/65536)*3.14159/180.0 * gyro_y;
      epson_data->gyro_z = (EPSON_GYRO_SF/65536)*3.14159/180.0 * gyro_z;
      idx += 12;
    }
    else
    {
      short gyro_x = (data[idx] << 8) + data[idx + 1];
      short gyro_y = (data[idx + 2] << 8) + data[idx + 3];
      short gyro_z = (data[idx + 4] << 8) + data[idx + 5];
      epson_data->gyro_x = EPSON_GYRO_SF*3.14159/180.0 * gyro_x;
      epson_data->gyro_y = EPSON_GYRO_SF*3.14159/180.0 * gyro_y;
      epson_data->gyro_z = EPSON_GYRO_SF*3.14159/180.0 * gyro_z;
      idx += 6;
    }
#ifdef DEBUG
    printf("gx: %8.5f\tgy: %8.5f\tgz: %8.5f\t", epson_data->gyro_x*180.0/3.14159, epson_data->gyro_y*180.0/3.14159, epson_data->gyro_z*180.0/3.14159);
#endif
  }

  if(options.accel_out)
  {
    if(options.accel_bit)
    {
      int accel_x = (data[idx] << 8*3) + (data[idx + 1] << 8*2) + (data[idx + 2] << 8) + data[idx + 3];
      int accel_y = (data[idx + 4] << 8*3) + (data[idx + 5] << 8*2) + (data[idx + 6] << 8) + data[idx + 7];
      int accel_z = (data[idx + 8] << 8*3) + (data[idx + 9] << 8*2) + (data[idx + 10] << 8) + data[idx + 11];
      epson_data->accel_x = (EPSON_ACCL_SF/65536)*9.80665/1000.0 * accel_x;
      epson_data->accel_y = (EPSON_ACCL_SF/65536)*9.80665/1000.0 * accel_y;
      epson_data->accel_z = (EPSON_ACCL_SF/65536)*9.80665/1000.0 * accel_z;
      idx += 12;
    }
    else
    {
      short accel_x = (data[idx] << 8) + data[idx + 1];
      short accel_y = (data[idx + 2] << 8) + data[idx + 3];
      short accel_z = (data[idx + 4] << 8) + data[idx + 5];
      epson_data->accel_x = (EPSON_ACCL_SF)*9.80665/1000.0 * accel_x;
      epson_data->accel_y = (EPSON_ACCL_SF)*9.80665/1000.0 * accel_y;
      epson_data->accel_z = (EPSON_ACCL_SF)*9.80665/1000.0 * accel_z;
      idx += 6;
    }
#ifdef DEBUG
    printf("ax: %8.5f\tay: %8.5f\taz: %8.5f\t", epson_data->accel_x*1000/9.80665, epson_data->accel_y*1000/9.80665, epson_data->accel_z*1000/9.80665);
#endif
  }

  if(options.gyro_delta_out)
  {
    if(options.gyro_delta_bit)
    {
      int gyro_delta_x = (data[idx] << 8*3) + (data[idx + 1] << 8*2) + (data[idx + 2] << 8) + data[idx + 3];
      int gyro_delta_y = (data[idx + 4] << 8*3) + (data[idx + 5] << 8*2) + (data[idx + 6] << 8) + data[idx + 7];
      int gyro_delta_z = (data[idx + 8] << 8*3) + (data[idx + 9] << 8*2) + (data[idx + 10] << 8) + data[idx + 11];
      double da_sf = da_scale_factors[options.dlt_range_ctrl];
      epson_data->gyro_delta_x = gyro_delta_x *(da_sf)/65536;
      epson_data->gyro_delta_y = gyro_delta_y *(da_sf)/65536;
      epson_data->gyro_delta_z = gyro_delta_z *(da_sf)/65536;
      idx += 12;
    }
    else
    {
      short gyro_delta_x = (data[idx] << 8) + data[idx + 1];
      short gyro_delta_y = (data[idx + 2] << 8) + data[idx + 3];
      short gyro_delta_z = (data[idx + 4] << 8) + data[idx + 5];
      double da_sf = da_scale_factors[options.dlt_range_ctrl];
      epson_data->gyro_delta_x = gyro_delta_x *(da_sf);
      epson_data->gyro_delta_y = gyro_delta_y *(da_sf);
      epson_data->gyro_delta_z = gyro_delta_z *(da_sf);
      idx += 6;
    }
#ifdef DEBUG
    printf("dax: %8.5f\tday: %8.5f\tdaz: %8.5f\t", epson_data->gyro_delta_x, epson_data->gyro_delta_y, epson_data->gyro_delta_z);
#endif
  }

  if(options.accel_delta_out)
  {
    if(options.accel_delta_bit)
    {
      int accel_delta_x = (data[idx] << 8*3) + (data[idx + 1] << 8*2) + (data[idx + 2] << 8) + data[idx + 3];
      int accel_delta_y = (data[idx + 4] << 8*3) + (data[idx + 5] << 8*2) + (data[idx + 6] << 8) + data[idx + 7];
      int accel_delta_z = (data[idx + 8] << 8*3) + (data[idx + 9] << 8*2) + (data[idx + 10] << 8) + data[idx + 11];
      double dv_sf = dv_scale_factors[options.dlt_range_ctrl];
      epson_data->accel_delta_x = accel_delta_x *(dv_sf)/65536;
      epson_data->accel_delta_y = accel_delta_y *(dv_sf)/65536;
      epson_data->accel_delta_z = accel_delta_z *(dv_sf)/65536;
      idx += 12;
    }
    else
    {
      short accel_delta_x = (data[idx] << 8) + data[idx + 1];
      short accel_delta_y = (data[idx + 2] << 8) + data[idx + 3];
      short accel_delta_z = (data[idx + 4] << 8) + data[idx + 5];
      double dv_sf = dv_scale_factors[options.dlt_range_ctrl];
      epson_data->accel_delta_x = accel_delta_x *(dv_sf);
      epson_data->accel_delta_y = accel_delta_y *(dv_sf);
      epson_data->accel_delta_z = accel_delta_z *(dv_sf);
      idx += 6;
    }
#ifdef DEBUG
    printf("dvx: %8.5f\tdvy: %8.5f\tdvz: %8.5f\t", epson_data->accel_delta_x, epson_data->accel_delta_y, epson_data->accel_delta_z);
#endif
  }


  if(options.atti_out)
  {
    if(options.atti_bit)
    {
      int roll =  (data[idx] << 8*3) + (data[idx + 1] << 8*2) + (data[idx + 2] << 8) + data[idx + 3];
      int pitch = (data[idx + 4] << 8*3) + (data[idx + 5] << 8*2) + (data[idx + 6] << 8) + data[idx + 7];
      int yaw =   (data[idx + 8] << 8*3) + (data[idx + 9] << 8*2) + (data[idx + 10] << 8) + data[idx + 11];
      epson_data->roll = (EPSON_ATTI_SF/65536)*3.14159/180.0 * roll;
      epson_data->pitch = (EPSON_ATTI_SF/65536)*3.14159/180.0 * pitch;
      epson_data->yaw = (EPSON_ATTI_SF/65536)*3.14159/180.0 * yaw;
      idx += 12;
    }
    else
    {
      short roll =  (data[idx] << 8) + data[idx + 1];
      short pitch = (data[idx + 2] << 8) + data[idx + 3];
      short yaw =   (data[idx + 4] << 8) + data[idx + 5];
      epson_data->roll = EPSON_ATTI_SF*3.14159/180.0 * roll;
      epson_data->pitch = EPSON_ATTI_SF*3.14159/180.0 * pitch;
      epson_data->yaw = EPSON_ATTI_SF*3.14159/180.0 * yaw;
      idx += 6;
    }
#ifdef DEBUG
    printf("roll: %8.3f\tpitch: %8.3f\tyaw: %8.3f\t", epson_data->roll*180.0/3.14159, epson_data->pitch*180.0/3.14159, epson_data->yaw*180.0/3.14159);
#endif
  }

  if(options.gpio_out)
  {
    unsigned short gpio = (data[idx] << 8) + data[idx + 1];
    epson_data->gpio = gpio;
    idx += 2;
#ifdef DEBUG
    printf("gpio: %04x\t", epson_data->gpio);
#endif
  }

#endif
  if(options.count_out)
  {
    int count = (data[idx] << 8) + data[idx + 1];
    if (options.ext_sel == 1)
      epson_data->count = count*EPSON_COUNT_SF;
    else
      epson_data->count = count;
#ifdef DEBUG
    printf("count: %09d\t", epson_data->count);
#endif
  }

}


/*****************************************************************************
** Function name:       sensorDataReadBurstNOptions
** Description:         Retrieves 1 packet from the incoming IMU stream based
**                      on expected burst length and searching for START and
**                      END markers. Then calls populateEpsonData() to
**                      post process into struct.
** Parameters:          options - struct describing IMU configuration.
**                      epson_data - struct that is filled with data.
** Return value:        SUCCESS or FAIL
** Notes:
******************************************************************************/
int sensorDataReadBurstNOptions(struct EpsonOptions options, struct EpsonData* epson_data)
{

  int byte_length = sensorDataByteLength(options);
  int data_length = byte_length - 2;    // exclude the START and END markers
  unsigned char byte;

  while(readComPort(comPort, &byte, 1) > 0)
  {
#ifdef DEBUG
    printf("state: %d, byte: 0x%02X\n", state, byte);
#endif
    // State machine to seek out START & END markers and then
    // call to populateEpsonData()
    switch(state)
    {
    case START:
      if(byte == UART_HEADER)
      state = DATA;
      break;
    case DATA:
      data[data_count] = byte;
      data_count++;
      if(data_count == data_length)
        state = END;
      break;
    case END:
      data_count = 0;
      state = START;
      if(byte == UART_DELIMITER)
      {
#ifdef DEBUG
        for(int i = 0; i < data_length; i++)
          printf("0x%02X ", data[i]);
        printf("\n");
#endif

#ifdef V340
        populateEpsonData(options, epson_data);
        return SUCCESS;

#endif
        if (options.checksum_out == 1)
        {
          unsigned short checksum = 0;
          for(int i = 0; i < data_length-2; i += 2)
          {
            checksum += (data[i] << 8) + data[i+1];
          }
          unsigned short epson_checksum = (data[data_length - 2] << 8) +
          data[data_length - 1];

          if(checksum == epson_checksum)
          {
            populateEpsonData(options, epson_data);
            return SUCCESS;
          }
          else
            printf("checksum failed\n");
        }
        else
        {
          populateEpsonData(options, epson_data);
          return SUCCESS;
        }
      }
      // Reaches here means checksum validation fails
      return FAIL;
      break;
    default:
      // Should never get here
      printf("Invalid State in Read Burst Processing\n");
    }
  }
  // No byte received in serial port yet
  return FAIL;
}
