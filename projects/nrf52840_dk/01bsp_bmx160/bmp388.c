/**
\brief This program shows the use of the "bmx160" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

This project configure bmx160 and read the gyroscope values in 3 axises.
Then it sends out the gyro data through uart at interval of SAMPLE_PERIOD.

\author Tengfei Chang <tengfei.chang@gmail.com>, Nov 2021.
*/

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "board.h"
#include "leds.h"
#include "sctimer.h"
#include "i2c.h"
#include "uart.h"
#include "bmp388.h"

//=========================== defines =========================================

#define SAMPLE_PERIOD   (32768>>1)  // @32kHz = 1s
#define BUFFER_SIZE     0x08   //2B*3 axises value + 2B ending with '\r\n'

//=========================== variables =======================================

typedef struct {
   uint16_t num_compare;
   bool sampling_now;
   uint8_t axes[6];
   float axis_x;
   float axis_y;
   float axis_z;

   uint8_t who_am_i;
   int16_t temp;
   float   temp_f;

   // uart specific
              uint8_t uart_lastTxByteIndex;
   volatile   uint8_t uartDone;
   volatile   uint8_t uartSendNow;
   volatile   uint8_t uartToSend[BUFFER_SIZE];
} app_vars_t;

app_vars_t app_vars;
struct bmp3_dev dev;

//=========================== prototypes ======================================

void cb_compare(void);
void cb_uartTxDone(void);
uint8_t cb_uartRxCb(void);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {

    int16_t tmp;
    uint8_t i;

    // initialize board. 
    board_init();

    // setup UART
    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();
   
    sctimer_set_callback(cb_compare);
    sctimer_setCompare(sctimer_readCounter()+SAMPLE_PERIOD);
         
    // alway set address first
    i2c_set_addr(BMP3_I2C_ADDR_PRIM);
    
    // should be 0xd8 for bmx160
    app_vars.who_am_i = bmp388_who_am_i();
  
    // configure bmp388
    bmp388_init();
    
    while (1) {

        // wait for timer to elapse
        while (app_vars.uartSendNow==0);
        app_vars.uartSendNow = 0;

        float pressure = readPressure();
        //float temperature = readTemperature();
        //float alt = readAltitude();

        i=0;
        app_vars.uartToSend[i++] = '\r';
        app_vars.uartToSend[i++] = '\n';

        // send string over UART
        app_vars.uartDone              = 0;
        app_vars.uart_lastTxByteIndex  = 0;
        uart_writeByte(app_vars.uartToSend[app_vars.uart_lastTxByteIndex]);
        while(app_vars.uartDone==0);
    }
}

//=========================== callbacks =======================================

void cb_compare(void) {
   
    // have main "task" send over UART
    app_vars.uartSendNow = 1;

    // schedule again
    sctimer_setCompare(sctimer_readCounter()+SAMPLE_PERIOD);
}

void cb_uartTxDone(void) {

    app_vars.uart_lastTxByteIndex++;
    if (app_vars.uart_lastTxByteIndex<sizeof(app_vars.uartToSend)) {
        uart_writeByte(app_vars.uartToSend[app_vars.uart_lastTxByteIndex]);
    } else {
        app_vars.uartDone = 1;
    }
}

uint8_t cb_uartRxCb(void) {
   
   uint8_t byte;
   
   // toggle LED
   leds_error_toggle();
   
   // read received byte
   byte = uart_readByte();
   
   // echo that byte over serial
   uart_writeByte(byte);
   
   return 0;
}

int8_t bmp388_read(uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
  if(reg_data == 0) {
    return -1;
  }
  i2c_read_bytes(reg_addr, reg_data, len);
  return 0;
}

int8_t bmp388_write(uint8_t *reg_addr, uint8_t *reg_data, uint8_t len)
{
  if(reg_data == 0) {
    return -1;
  }
  i2c_write_bytes(*reg_addr, reg_data, len);
  return 0;
}


uint8_t bmp388_who_am_i(void)
{
  uint8_t chip_id=0;
  int8_t ret = bmp388_read(BMP3_CHIP_ID_ADDR, &chip_id, 1);
  return chip_id;
}

void parse_calib_data(const uint8_t *reg_data)
{
  /* Temporary variable to store the aligned trim data */
  struct bmp3_reg_calib_data *reg_calib_data = &dev.calib_data.reg_calib_data;
  struct bmp3_quantized_calib_data *quantized_calib_data = &dev.calib_data.quantized_calib_data;
  /* Temporary variable */
  double temp_var;

  /* 1 / 2^8 */
  temp_var = 0.00390625f;
  reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
  quantized_calib_data->par_t1 = ((double)reg_calib_data->par_t1 / temp_var);

  reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
  temp_var = 1073741824.0f;
  quantized_calib_data->par_t2 = ((double)reg_calib_data->par_t2 / temp_var);

  reg_calib_data->par_t3 = (int8_t)reg_data[4];
  temp_var = 281474976710656.0f;
  quantized_calib_data->par_t3 = ((double)reg_calib_data->par_t3 / temp_var);

  reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);

  temp_var = 1048576.0f;
  quantized_calib_data->par_p1 = ((double)(reg_calib_data->par_p1 - (16384)) / temp_var);

  reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
  temp_var = 536870912.0f;
  quantized_calib_data->par_p2 = ((double)(reg_calib_data->par_p2 - (16384)) / temp_var);

  reg_calib_data->par_p3 = (int8_t)reg_data[9];
  temp_var = 4294967296.0f;
  quantized_calib_data->par_p3 = ((double)reg_calib_data->par_p3 / temp_var);

  reg_calib_data->par_p4 = (int8_t)reg_data[10];
  temp_var = 137438953472.0f;
  quantized_calib_data->par_p4 = ((double)reg_calib_data->par_p4 / temp_var);

  reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
  /* 1 / 2^3 */
  temp_var = 0.125f;
  quantized_calib_data->par_p5 = ((double)reg_calib_data->par_p5 / temp_var);

  reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14],  reg_data[13]);
  temp_var = 64.0f;
  quantized_calib_data->par_p6 = ((double)reg_calib_data->par_p6 / temp_var);

  reg_calib_data->par_p7 = (int8_t)reg_data[15];
  temp_var = 256.0f;
  quantized_calib_data->par_p7 = ((double)reg_calib_data->par_p7 / temp_var);

  reg_calib_data->par_p8 = (int8_t)reg_data[16];
  temp_var = 32768.0f;
  quantized_calib_data->par_p8 = ((double)reg_calib_data->par_p8 / temp_var);

  reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
  temp_var = 281474976710656.0f;
  quantized_calib_data->par_p9 = ((double)reg_calib_data->par_p9 / temp_var);

  reg_calib_data->par_p10 = (int8_t)reg_data[19];
  temp_var = 281474976710656.0f;
  quantized_calib_data->par_p10 = ((double)reg_calib_data->par_p10 / temp_var);

  reg_calib_data->par_p11 = (int8_t)reg_data[20];
  temp_var = 36893488147419103232.0f;
  quantized_calib_data->par_p11 = ((double)reg_calib_data->par_p11 / temp_var);
}

int8_t bmp388_get_calib_data(void)
{
  int8_t result;
  uint8_t calib_data[BMP3_CALIB_DATA_LEN] = {0};
  result = bmp388_read(BMP3_CALIB_DATA_ADDR, calib_data, BMP3_CALIB_DATA_LEN);
  parse_calib_data(calib_data);
  return result;
}

int8_t set_pwr_ctrl_settings(uint32_t desired_settings)
{
  int8_t rslt;
  uint8_t reg_addr = BMP3_PWR_CTRL_ADDR;
  uint8_t reg_data;

  rslt = bmp388_read(reg_addr, &reg_data, 1);

  if (rslt == BMP3_OK) {
    if (desired_settings & BMP3_PRESS_EN_SEL) {
      /* Set the pressure enable settings in the
      register variable */
      reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_PRESS_EN, dev.settings.press_en);
    }
    if (desired_settings & BMP3_TEMP_EN_SEL) {
      /* Set the temperature enable settings in the
      register variable */
      reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, dev.settings.temp_en);
    }
    /* Write the power control settings in the register */
    rslt = bmp388_write(&reg_addr, &reg_data, 1);
  }

  return rslt;
}

int8_t bmp388_set_sensor_settings(uint32_t desired_settings)
{
  int8_t rslt;
  if (POWER_CNTL&desired_settings) {
    /* Set the power control settings */
    rslt = set_pwr_ctrl_settings(desired_settings);
  }
  return rslt;
}

int8_t write_power_mode(void)
{
  int8_t rslt;
  uint8_t reg_addr = BMP3_PWR_CTRL_ADDR;
  uint8_t op_mode = dev.settings.op_mode;
  /* Temporary variable to store the value read from op-mode register */
  uint8_t op_mode_reg_val;

  /* Read the power mode register */
  rslt = bmp388_read(reg_addr, &op_mode_reg_val, 1);
  /* Set the power mode */
  if (rslt == BMP3_OK) {
    op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, op_mode);
    /* Write the power mode in the register */
    rslt = bmp388_write(&reg_addr, &op_mode_reg_val, 1);
  }

  return rslt;
}

int8_t bmp388_set_op_mode(void)
{
  int8_t rslt;
  rslt = write_power_mode();
  return rslt;
}

int8_t bmp388_set_config(void)
{
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    dev.settings.press_en = BMP3_ENABLE;
    dev.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    dev.settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
    dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    dev.settings.odr_filter.odr = BMP3_ODR_200_HZ;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
    rslt = bmp388_set_sensor_settings(settings_sel);

    /* Set the power mode to normal mode */
    dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp388_set_op_mode();

    return rslt;
}

void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data)
{
  /* Temporary variables to store the sensor data */
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;

  /* Store the parsed register values for pressure data */
  data_xlsb = (uint32_t)reg_data[0];
  data_lsb = (uint32_t)reg_data[1] << 8;
  data_msb = (uint32_t)reg_data[2] << 16;
  uncomp_data->pressure = data_msb | data_lsb | data_xlsb;
  //Serial.println(uncomp_data->pressure);
  /* Store the parsed register values for temperature data */
  data_xlsb = (uint32_t)reg_data[3];
  data_lsb = (uint32_t)reg_data[4] << 8;
  data_msb = (uint32_t)reg_data[5] << 16;
  uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
  //Serial.println(uncomp_data->temperature);
}

double bmp3_pow(double base, uint8_t power)
{
  double pow_output = 1;

  while (power != 0) {
    pow_output = base * pow_output;
    power--;
  }

  return pow_output;
}


double compensate_pressure(const struct bmp3_uncomp_data *uncomp_data,
          const struct bmp3_calib_data *calib_data)
{
  const struct bmp3_quantized_calib_data *quantized_calib_data = &calib_data->quantized_calib_data;
  /* Variable to store the compensated pressure */
  double comp_press;
  /* Temporary variables used for compensation */
  double partial_data1;
  double partial_data2;
  double partial_data3;
  double partial_data4;
  double partial_out1;
  double partial_out2;
  
  partial_data1 = quantized_calib_data->par_p6 * quantized_calib_data->t_lin;
  partial_data2 = quantized_calib_data->par_p7 * bmp3_pow(quantized_calib_data->t_lin, 2);
  partial_data3 = quantized_calib_data->par_p8 * bmp3_pow(quantized_calib_data->t_lin, 3);
  partial_out1 = quantized_calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
  
  
  partial_data1 = quantized_calib_data->par_p2 * quantized_calib_data->t_lin;
  partial_data2 = quantized_calib_data->par_p3 * bmp3_pow(quantized_calib_data->t_lin, 2);
  partial_data3 = quantized_calib_data->par_p4 * bmp3_pow(quantized_calib_data->t_lin, 3);
  partial_out2 = uncomp_data->pressure *
      (quantized_calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
      
  
  
  partial_data1 = bmp3_pow((double)uncomp_data->pressure, 2);
  partial_data2 = quantized_calib_data->par_p9 + quantized_calib_data->par_p10 * quantized_calib_data->t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + bmp3_pow((double)uncomp_data->pressure, 3) * quantized_calib_data->par_p11;
  comp_press = partial_out1 + partial_out2 + partial_data4;
  return comp_press;
}

double compensate_temperature(const struct bmp3_uncomp_data *uncomp_data,
            struct bmp3_calib_data *calib_data)
{
  uint32_t uncomp_temp = uncomp_data->temperature;
  double partial_data1;
  double partial_data2;

  partial_data1 = (double)(uncomp_temp - calib_data->quantized_calib_data.par_t1);
  partial_data2 = (double)(partial_data1 * calib_data->quantized_calib_data.par_t2);
  /* Update the compensated temperature in calib structure since this is
     needed for pressure calculation */
  calib_data->quantized_calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1)
              * calib_data->quantized_calib_data.par_t3;

  /* Return compensated temperature */
  return calib_data->quantized_calib_data.t_lin;
}

int8_t compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data,
             struct bmp3_data *comp_data, struct bmp3_calib_data *calib_data)
{
  int8_t rslt = BMP3_OK;

  if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)) {
    /* If pressure or temperature component is selected */
    if (sensor_comp & (BMP3_PRESS | BMP3_TEMP)) {
      /* Compensate the temperature data */
      comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
    }
    if (sensor_comp & BMP3_PRESS) {
      /* Compensate the pressure data */
      comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
    }
  } else {
    rslt = BMP3_E_NULL_PTR;
  }

  return rslt;
}

int8_t bmp388_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data)
{
  int8_t rslt;
  /* Array to store the pressure and temperature data read from
  the sensor */
  uint8_t reg_data[BMP3_P_T_DATA_LEN] = {0};
  struct bmp3_uncomp_data uncomp_data = {0};
  if ((comp_data != NULL)) {
    /* Read the pressure and temperature data from the sensor */
    rslt = bmp388_read(BMP3_DATA_ADDR, reg_data, BMP3_P_T_DATA_LEN);
    if (rslt == BMP3_OK) {
      /* Parse the read data from the sensor */
      parse_sensor_data(reg_data, &uncomp_data);
      /* Compensate the pressure/temperature/both data read
         from the sensor */
      rslt = compensate_data(sensor_comp, &uncomp_data, comp_data, &dev.calib_data);
    }
  } else {
    rslt = BMP3_E_NULL_PTR;
  }

  return rslt;
}

float readPressure(void){
  uint8_t sensor_comp;
  sensor_comp = BMP3_PRESS;
  struct bmp3_data data;
  bmp388_get_sensor_data(sensor_comp, &data);
  return data.pressure;
}

float readTemperature(void){
  uint8_t sensor_comp;
  sensor_comp = BMP3_TEMP;
  struct bmp3_data data;
  bmp388_get_sensor_data(sensor_comp, &data);
  return data.temperature;
}

float readAltitude(void)
{
  float pressure = readPressure();
  return (1.0 - bmp3_pow(pressure / 101325, 0.190284)) * 287.15 / 0.0065;
}

int8_t bmp388_reset(void)
{
  int8_t rslt;
  uint8_t reg_addr = BMP3_CMD_ADDR;
  /* 0xB6 is the soft reset command */
  uint8_t soft_rst_cmd = 0xB6;
  uint8_t cmd_rdy_status;
  uint8_t cmd_err_status;

  /* Check for command ready status */
  rslt = bmp388_read(BMP3_SENS_STATUS_REG_ADDR, &cmd_rdy_status, 1);
  /* Device is ready to accept new command */
  if ((cmd_rdy_status & BMP3_CMD_RDY) && (rslt == BMP3_OK)) {
    /* Write the soft reset command in the sensor */
    rslt = bmp388_write(&reg_addr, &soft_rst_cmd, 1);
    /* Proceed if everything is fine until now */
    
    if (rslt == BMP3_OK) {
      /* Read for command error status */
      rslt = bmp388_read(BMP3_ERR_REG_ADDR, &cmd_err_status, 1);
      /* check for command error status */
      if ((cmd_err_status & BMP3_CMD_ERR) || (rslt != BMP3_OK)) {
        /* Command not written hence return
           error */
        rslt = BMP3_E_CMD_EXEC_FAILED;
      }
    }
  } else {
    rslt = BMP3_E_CMD_EXEC_FAILED;
  }
  return rslt;
}

int8_t bmp388_init(void)
{
  int8_t result = bmp388_reset();
  if (result == BMP3_OK) {
    result = bmp388_get_calib_data();
  }
  result = bmp388_set_config();
  return result;
}