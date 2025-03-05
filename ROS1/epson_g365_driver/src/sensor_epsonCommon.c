//==============================================================================
//
//  sensor_epsonCommon.c - Epson IMU sensor protocol specific code common 
//                      for all IMU models
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


/*****************************************************************************
** Function name:       sensorHWReset
** Description:         Toggle the RESET pin, delay, wait for NOT_READY bit=0
**                      This is only applicable on embedded platforms with 
**                      GPIO pin connected to IMU RESET#
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int sensorHWReset( void )
{
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  gpioSet(EPSON_RESET);      // RESET pin HIGH
  seDelayMS(DELAY_EPSON_RESET);
  gpioClr(EPSON_RESET);      // Assert RESET (LOW)
  seDelayMS(DELAY_EPSON_RESET);
  gpioSet(EPSON_RESET);      // Deassert RESET (HIGH)
  seDelayMS(EPSON_POWER_ON_DELAY);

  // Poll NOT_READY bit every 1msec until returns 0
  // Exit after specified retries
  do
  {
    rxData = registerRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, debug);
    seDelayMicroSecs(1000);
    retryCount--;
  } while((rxData & 0x0400) == 0x0400 && (retryCount != 0));

  if (retryCount == 0)
  {
    printf("\r\n...Error: NOT_READY stuck HIGH.");
    return FAIL;
  }
  return SUCCESS;
}


/*****************************************************************************
** Function name:       sensorPowerOn
** Description:         Initial startup, Goto Config Mode (sanity), 
**                      Trigger HW Reset, check for Hardware Error Flags
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int sensorPowerOn(void)
{
  unsigned short rxData = 0xFFFF;
  unsigned int debug = FALSE;
  unsigned short retryCount = 3000;

  // Safety Measure, Force Exit of Sampling Mode
  do
  {
    registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING, debug);
    rxData = registerRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, debug);
    seDelayMicroSecs(1000);
    retryCount--;
  } while((rxData & 0x0400) == 0x0000 && (retryCount != 0));

  if (retryCount == 0)
  {
    printf("\r\n...Error: Stuck in Sampling Mode.");
    return FAIL;
  }

  // Hardware Reset if connected, and check for NOT_READY flag
  if (!sensorHWReset())
    return FAIL;

  // Check for error flags
  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  if (rxData == 0x0000)
    return SUCCESS;
  else
    return FAIL;
}



/*****************************************************************************
** Function name:       sensorStart
** Description:         Start sensor sampling (goto Sampling Mode)
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorStart(void)
{
  unsigned int debug = FALSE;

  registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_BEGIN_SAMPLING, debug);
  printf("\r\n...Sensor start.");
}


/*****************************************************************************
** Function name:       sensorStop
** Description:         Stop sensor sampling (goto Config Mode)
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorStop(void)
{
  unsigned int debug = FALSE;

  registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING, debug);
  seDelayMicroSecs(200000);	// Provide 200msec for sensor to finish sending sample
  printf("\r\n...Sensor stop.");
}


/*****************************************************************************
** Function name:       sensorReset
** Description:         Send Software Reset to Sensor + Delay 800 msec
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorReset(void)
{
  unsigned int debug = FALSE;

  printf("\r\n...Software Reset begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_SOFTRESET, debug);
  seDelayMS(EPSON_POWER_ON_DELAY);
  printf("\r\n...Software Reset complete.");
}


/*****************************************************************************
** Function name:       sensorFlashTest
** Description:         Send Flashtest command to Sensor and check status
**                      NOTE: Not supported for V340
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int sensorFlashTest(void)
{

#if defined V340
  // always return SUCCESS
#else
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...Flash test begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_FLASHTEST, debug);
  seDelayMS(EPSON_FLASH_TEST_DELAY);
  do
  {
    rxData = registerRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while((rxData & 0x0800) == 0x0800 && (retryCount != 0));
  if (retryCount == 0)
  {
    printf("\r\n...Error: Flashtest bit did not return to 0b.");
    return FAIL;
  }

  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Flash test complete.");

  if ((rxData & 0x0004) != 0x0000)
    return FAIL;

#endif
  return SUCCESS;

}


/*****************************************************************************
** Function name:       sensorSelfTest
** Description:         Send SelfTest command to Sensor and check status
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int sensorSelfTest(void)
{
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...Self test begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_SELFTEST, debug);
  seDelayMS(EPSON_SELF_TEST_DELAY);
  do
  {
    rxData = registerRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while((rxData & 0x0400) == 0x0400 && (retryCount != 0));
  if (retryCount == 0)
  {
    printf("\r\n...Error: Self test bit did not return to 0b.");
    return FAIL;
  }

  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Self test complete.");

  if ((rxData & 0x0002) == 0x0000)
    return SUCCESS;
  else
    return FAIL;

}


/*****************************************************************************
** Function name:       sensorInitialBackup
** Description:         Send InitialBackup command (restore defaults) to Sensor 
**                      and check status
**                      NOTE: Only supported for G365/G370
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int sensorInitialBackup(void)
{
#if defined G354 || defined G364PDCA || defined G364PDC0 || defined V340 || defined G320
  // always return SUCCESS
#else
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...InitialBackup begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_INITIAL_BACKUP, debug);
  seDelayMS(EPSON_FLASH_TEST_DELAY);

  do
  {
    rxData = registerRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, debug);
    retryCount--;
  } while((rxData & 0x0010) == 0x0010 && (retryCount != 0));
  if (retryCount == 0)
  {
    printf("\r\n...Error: InitialBackup bit did not return to 0b.");
    return FAIL;
  }

  printf("\r\n...Initial Backup complete.");
#endif
  return SUCCESS;
}



/*****************************************************************************
** Function name:       sensorInitOptions
** Description:         Initialize the sensor hardware to desired settings
**                      based on EpsonOptions
** Parameters:          struct EpsonOptions
** Return value:        SUCCESS or FAIL
**
*****************************************************************************/
int sensorInitOptions(struct EpsonOptions options)
{
  unsigned int debug = FALSE;

  // SIG_CTRL
  // ND flags for gyro_delta_out X,Y,Z are enabled if gyro_delta_out is enabled
  // ND flags for accel_delta_out X,Y,Z are enabled if accel_delta_out is enabled

  int sig_ctrl_lo =       (options.gyro_delta_out & 0x01) << 2 |
                          (options.gyro_delta_out & 0x01) << 3 |
                          (options.gyro_delta_out & 0x01) << 4 |
                          (options.accel_delta_out & 0x01) << 5 |
                          (options.accel_delta_out & 0x01) << 6 |
                          (options.accel_delta_out & 0x01) << 7;

  // ND flags for gyro_out X,Y,Z are enabled if gyro_out is enabled
  // ND flags for accel_out X,Y,Z are enabled if accel_out is enabled
  // ND flag for temp_out is enabled if temp_out is enabled

  int sig_ctrl_hi =       (options.accel_out & 0x01) << 1 |
                          (options.accel_out & 0x01) << 2 |
                          (options.accel_out & 0x01) << 3 |
                          (options.gyro_out & 0x01) << 4 |
                          (options.gyro_out & 0x01) << 5 |
                          (options.gyro_out & 0x01) << 6 |
                          (options.temp_out & 0x01) << 7;

  // MSC_CTRL
  // Configure DRDY function (if needed) & EXT pin function on GPIO2 (if needed)
  // External Counter Reset is typically used when GPIO2 is connected to a PPS-like signal

  int msc_ctrl_lo =       (options.drdy_pol & 0x01) << 1 |
                          (options.drdy_on & 0x01) << 2 |
                          (options.ext_pol & 0x01) << 5 |
                          (options.ext_sel & 0x03) << 6;

  // SMPL_CTRL
  // Configures the Data Output Rate of the IMU.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations

  int smpl_ctrl_hi =      (options.dout_rate & 0x0F);

  // FILTER_CTRL
  // Configures the FIR filter of the IMU.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  // If External Trigger is enabled on GPIO2, then it is recommended to set the
  // the FILTER_SEL=0. And program the GYRO_LPF_FC & ACCL_LPF_FC to meet Nyquist
  // based on the Trigger Frequency.

  int filter_ctrl_lo =    (options.filter_sel & 0x1F);

#if defined V340
  // COUNT_CTRL
  // V340 only has option to enable or disable counter with burst read packets
  // Otherwise the packet format and data fields are fixed

  int count_ctrl_lo =     (options.count_out & 0x1);
  
#elif defined G365PDC0 || defined G365PDF0 || defined G325PDF0
  // BURST_CTRL1
  // These enable or disable certain data fields in the burst read packet
  // For G365 or G325 specifically, attitude output can be enabled or disabled.

  int burst_ctrl1_lo =    (options.checksum_out & 0x1) |
                          (options.count_out & 0x1) << 1 |
                          (options.gpio_out & 0x01) << 2;

  int burst_ctrl1_hi =    (options.atti_out & 0x1) |
                          (options.gyro_delta_out & 0x1) << 2 |
                          (options.accel_delta_out & 0x01) << 3 |
                          (options.accel_out & 0x01) << 4 |
                          (options.gyro_out & 0x01) << 5 |
                          (options.temp_out & 0x01) << 6 |
                          (options.flag_out & 0x01) << 7;

  // BURST_CTRL2
  // If certain data fields are enabled, these bits determine if the
  // data fields are 16 or 32 bit

  int burst_ctrl2_hi =    (options.atti_bit & 0x1) |
                          (options.gyro_delta_bit & 0x01) << 2 |
                          (options.accel_delta_bit & 0x01) << 3 |
                          (options.accel_bit & 0x01) << 4 |
                          (options.gyro_bit & 0x01) << 5 |
                          (options.temp_bit & 0x01) << 6;

  // DLT_CTRL
  // Enable or disable Delta Angle/Velocity overflow flag in DIAG_STAT
  // Set the Delta Angle/Velocity Scale Factor

  int dlt_ctrl_hi =       (options.dlt_ovf_en & 0x01) << 1;
  int dlt_ctrl_lo =       (options.dlt_range_ctrl & 0x0F); 

  // ATTI_CTRL
  // Attitude Output & Delta Angle/Velocity functions are mutually exclusive.
  // User should only enable one or the other, but not both.
  // Attitude Mode = 0=Inclination 1=Euler

  int atti_ctrl_hi =      (options.gyro_delta_out & 0x01) << 1 |
                          (options.atti_out & 0x01) << 2 |
                          (options.atti_mode & 0x01) << 3 ;

  // ATTI_CTRL
  // Refer to datasheet to determine the different Euler Angle configurations

  int atti_ctrl_lo =      (options.atti_conv & 0x1f);

  // LPF_CTRL
  // Refer to the datasheet to determine the Low Pass Filter settings for Gyro & Accel
  // These typically are only used when External Trigger option is enabled
  // for maintaining Nyquist. The cutoff frequency should be set to 1/2 the external
  // trigger frequency

  int lpf_ctrl_lo =       (options.gyro_lpf_fc & 0x7);
  int lpf_ctrl_hi =       (options.accl_lpf_fc & 0x7);


#elif defined G370PDC0 || defined G370PDF0
  // BURST_CTRL1
  // These enable or disable certain data fields in the burst read packet

  int burst_ctrl1_lo =    (options.checksum_out & 0x1) |
                          (options.count_out & 0x1) << 1 |
                          (options.gpio_out & 0x01) << 2;

  int burst_ctrl1_hi =    (options.gyro_delta_out & 0x1) << 2 |
                          (options.accel_delta_out & 0x01) << 3 |
                          (options.accel_out & 0x01) << 4 |
                          (options.gyro_out & 0x01) << 5 |
                          (options.temp_out & 0x01) << 6 |
                          (options.flag_out & 0x01) << 7;

  // BURST_CTRL2
  // If certain data fields are enabled, these bits determine if the
  // data fields are 16 or 32 bit

  int burst_ctrl2_hi =    (options.gyro_delta_bit & 0x01) << 2 |
                          (options.accel_delta_bit & 0x01) << 3 |
                          (options.accel_bit & 0x01) << 4 |
                          (options.gyro_bit & 0x01) << 5 |
                          (options.temp_bit & 0x01) << 6;

  // DLT_CTRL
  // Enable or disable Delta Angle/Velocity overflow flag in DIAG_STAT
  // Set the Delta Angle/Velocity Scale Factor

  int dlt_ctrl_hi =       (options.dlt_ovf_en & 0x01) << 1;
  int dlt_ctrl_lo =       (options.dlt_range_ctrl & 0x0F); 

  // ATTI_CTRL
  // For G370, Attitude Output is not supported.
  // This is only for Delta Angle/Velocity function if enabled.

  int atti_ctrl_hi =      (options.gyro_delta_out & 0x01) << 1;

  // ATTI_CTRL
  // Actually not used for G370

  int atti_ctrl_lo = 0x00;

  // LPF_CTRL
  // Refer to the datasheet to determine the Low Pass Filter settings for Gyro & Accel
  // These typically are only used when External Trigger option is enabled
  // for maintaining Nyquist. The cutoff frequency should be set to 1/2 the external
  // trigger frequency

  int lpf_ctrl_lo =       (options.gyro_lpf_fc & 0x7);
  int lpf_ctrl_hi =       (options.accl_lpf_fc & 0x7);

#else  // G354/G364/G320
  // BURST_CTRL1
  // These enable or disable certain data fields in the burst read packet

  int burst_ctrl1_lo =    (options.checksum_out & 0x1) |
                          (options.count_out & 0x1) << 1 |
                          (options.gpio_out & 0x01) << 2;

  int burst_ctrl1_hi =    (options.accel_out & 0x01) << 4 |
                          (options.gyro_out & 0x01) << 5 |
                          (options.temp_out & 0x01) << 6 |
                          (options.flag_out & 0x01) << 7;

  // BURST_CTRL2
  // If certain data fields are enabled, these bits determine if the
  // data fields are 16 or 32 bit

  int burst_ctrl2_hi =    (options.accel_bit & 0x01) << 4 |
                          (options.gyro_bit & 0x01) << 5 |
                          (options.temp_bit & 0x01) << 6;
#endif
  
  // POL_CTRL
  // If these bits are set, then the axis values are reverse polarity

  int pol_ctrl_lo =       (options.invert_zaccel & 0x01) << 1 |
                          (options.invert_yaccel & 0x01) << 2 |
                          (options.invert_xaccel & 0x01) << 3 |
                          (options.invert_zgyro & 0x01) << 4 |
                          (options.invert_ygyro & 0x01) << 5 |
                          (options.invert_xgyro & 0x01) << 6;

#ifdef DEBUG
  printf("%02x: %02x\n%02x: %02x\n%02x: %02x\n%02x: %02x\n%02x: %02x\n%02x: %02x\n%02x: %02x\n%02x: %02x\n%02x: %02x\n",
  addr_sig_ctrl_hi, sig_ctrl_hi,
  addr_msc_ctrl_lo, msc_ctrl_lo,
  addr_smpl_ctrl_hi, smpl_ctrl_hi,
  addr_pol_ctrl_lo, pol_ctrl_lo,
  addr_filter_ctrl_lo, filter_ctrl_lo,
  addr_uart_ctrl_lo, uart_ctrl_lo,
  addr_burst_ctrl1_lo, burst_ctrl1_lo,
  addr_burst_ctrl1_hi, burst_ctrl1_hi,
  addr_burst_ctrl2_hi, burst_ctrl2_hi);
#endif

  registerWriteByte(CMD_WINDOW1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, debug);

#if defined G365PDC0 || defined G365PDF0 || defined G370PDC0 || defined G370PDF0 || G325PDF0
  registerWriteByte(CMD_WINDOW1, ADDR_SIG_CTRL_LO, sig_ctrl_lo, debug);
#endif

  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_LO, msc_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_POL_CTRL_LO, pol_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filter_ctrl_lo, debug);

#if !defined V340  // All models except V340

  // Delay for filter config
  seDelayMS(EPSON_FILTER_DELAY);

  // Check that the FILTER_BUSY bit returns 0
  unsigned short rxData;
  unsigned short retryCount = 3000;
  do
  {
    rxData = registerRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, debug);
    retryCount--;
  } while((rxData & 0x0020) == 0x0020 && (retryCount != 0));

  if (retryCount == 0)
  {
    printf("\r\n...Error: Filter busy bit did not return to 0b.");
    return FALSE;
  }
#endif

  // Always enable UART_AUTO mode for burst reading
  registerWriteByte(CMD_WINDOW1, ADDR_UART_CTRL_LO, 0x01, debug);

#if defined V340
  registerWriteByte(CMD_WINDOW1, ADDR_COUNT_CTRL_LO, count_ctrl_lo, debug);

#else
  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, burst_ctrl1_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL1_HI, burst_ctrl1_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL2_HI, burst_ctrl2_hi, debug);
#endif

#if defined G365PDC0 || defined G365PDF0 || defined G370PDC0 || defined G370PDF0 || defined G325PDF0
  registerWriteByte(CMD_WINDOW1, ADDR_DLT_CTRL_HI, dlt_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_DLT_CTRL_LO, dlt_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_ATTI_CTRL_HI, atti_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_ATTI_CTRL_LO, atti_ctrl_lo, debug);
  // Currently this software does not write to these LPF registers
  // They should only be used when enabling External Trigger mode
  // Uncomment the lines below if External Trigger is to be used
  //registerWriteByte(CMD_WINDOW1, ADDR_LPF_CTRL_HI, lpf_ctrl_hi, debug); 
  //registerWriteByte(CMD_WINDOW1, ADDR_LPF_CTRL_LO, lpf_ctrl_lo, debug);

#endif

  return TRUE;
}


/*****************************************************************************
** Function name:       sensorDummyWrite
** Description:         Sets the WINDOW_ID of IMU to 0
**                      This is a workaround to flush the UART port on embedded 
**                      Linux platform to prevent hanging if the first register
**                      access is register read
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorDummyWrite(void)
{
  unsigned int debug = FALSE;

  seDelayMicroSecs(100000);
  registerWriteByte(CMD_WINDOW0, ADDR_WIN_CTRL, 0x00, debug);
  seDelayMicroSecs(100000);
  printf("\r\n...sensorDummyWrite.");
}
