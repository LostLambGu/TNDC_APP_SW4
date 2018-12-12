/*******************************************************************************
* File Name          : lis2dh_driver.c
* Author             : Yangjie Gu
* Description        : This file provides all the lis2dh_driver functions.
* History:
*  09/30/2017 : lis2dh_driver V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

#include "lis2dh_driver.h"
#include "uart_api.h"
#include "rtcclock.h"
#include "initialization.h"
#include "common.h"

#if GSENSOR_I2C_USE_GPIO_SIMULATION
#include "i2c_driver.h"
#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*comment to eliminate errors*/
/*comment atel_ related lines*/
#define Lis2dhPrintf DebugPrintf

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define GSENSOR_INT1_PORT PB1_ACC_IRQ_GPIO_Port
#define GSENSOR_INT1_PIN PB1_ACC_IRQ_Pin

LIS2DH_POSITION_6D_t gsensordirection = LIS2DH_UNKOWN;
LIS2DH_POSITION_6D_t old_gsensordirection = LIS2DH_UNKOWN;

static __IO uint8_t AccIntHappenStatus = FALSE;
uint8_t Lis2dhMemsChipID = 0;

extern volatile unsigned int i2c2Lock;
/*----------------------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
#if (!GSENSOR_I2C_USE_GPIO_SIMULATION)
extern I2C_HandleTypeDef hi2c2;
#define LIS2DH_I2C_HANDLE (hi2c2)
#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */
/* Private function prototypes -----------------------------------------------*/

void DelayLis2dhUsTime(uint16_t time) //delay 5+2*time (machine time)
{
  uint16_t Count;
  Count = time;
  while (--Count)
    ;
  while (--time)
    ;
}

// DelayMsTime
void DelayLis2dhMsTime(uint16_t delay_time) //delay delay_time * 1ms
{
  uint16_t i;
  uint8_t j;

  if (delay_time == 0)
    return;

  for (i = 0; i < delay_time; i++)
  {
    for (j = 0; j < 10; j++)
    {
      DelayLis2dhUsTime(155);
    }
  }
}

void SetAccIntHappenStatus(uint8_t status)
{
	AccIntHappenStatus = status;
}

uint8_t GetAccIntHappenStatus(void)
{
	return AccIntHappenStatus;
}

void GSensorI2cInit(void)
{
  Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] LIS2DH: Enable", FmtTimeShow());
  //Inizialize MEMS Sensor
  //set ODR (turn ON device)
  LIS2DH_SetODR(LIS2DH_ODR_100Hz);
  //set PowerMode
  LIS2DH_SetMode(LIS2DH_NORMAL);
  //set Fullscale
  LIS2DH_SetFullScale(LIS2DH_FULLSCALE_2);
  //set axis Enable
  LIS2DH_SetAxis(LIS2DH_X_ENABLE | LIS2DH_Y_ENABLE | LIS2DH_Z_ENABLE);

  //LIS2DH_SetBDU(MEMS_ENABLE);
  //set Int1Pin
  LIS2DH_SetInt1Pin(LIS2DH_I1_INT1_ON_PIN_INT1_ENABLE);

  //LIS2DH_Int1LatchEnable(MEMS_ENABLE);
  LIS2DH_HPFAOI1Enable(MEMS_ENABLE);
  LIS2DH_SetInt1Threshold(10);
  // LIS2DH_SetInt1Duration(5);
  //LIS2DH_ResetInt1Latch();
  LIS2DH_SetIntConfiguration(LIS2DH_INT1_ZHIE_ENABLE | LIS2DH_INT1_ZLIE_ENABLE |
                             LIS2DH_INT1_YHIE_ENABLE | LIS2DH_INT1_YLIE_ENABLE |
                             LIS2DH_INT1_XHIE_ENABLE | LIS2DH_INT1_XLIE_ENABLE);
  //LIS2DH_SetInt6D4DConfiguration(LIS2DH_INT1_6D_ENABLE);
  LIS2DH_SetIntMode(LIS2DH_INT_MODE_6D_POSITION);
}

void AccelWriteRegister(uint8_t address, uint8_t reg, uint8_t val)
{
  GetExclusiveLock(&i2c2Lock);
  #if GSENSOR_I2C_USE_GPIO_SIMULATION
  ExtI2cWriteRegister(address, reg, val);
  #else
  HAL_StatusTypeDef res;

  res = HAL_I2C_Mem_Write(&LIS2DH_I2C_HANDLE, (uint16_t)address, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&val, 1, 1000);
  if (res != HAL_OK)
  {
    Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] LIS2DH: Write Byte Fail", FmtTimeShow());
  }
  #endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */
  FreeExclusiveLock(&i2c2Lock);
}

uint8_t AccelReadRegister(uint8_t address, uint8_t reg)
{
  uint8_t rdata = 0;
  GetExclusiveLock(&i2c2Lock);
  #if GSENSOR_I2C_USE_GPIO_SIMULATION
  rdata = ExtI2cReadRegister(address, reg);
  #else
  HAL_StatusTypeDef res;

  res = HAL_I2C_Mem_Read(&LIS2DH_I2C_HANDLE, (uint16_t)address, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&rdata, 1, 1000);
  if (res != HAL_OK)
  {
    Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] LIS2DH: Read Byte Fail", FmtTimeShow());
  }
  #endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */
  FreeExclusiveLock(&i2c2Lock);
  return (rdata);
}

void AcceReadSerialData(uint8_t address, uint8_t reg, uint8_t count, uint8_t *buff)
{
  GetExclusiveLock(&i2c2Lock);
  #if GSENSOR_I2C_USE_GPIO_SIMULATION
  ExtI2cReadSerialData(address, reg, count, buff);
  #else
  HAL_StatusTypeDef res;

  res = HAL_I2C_Mem_Read(&LIS2DH_I2C_HANDLE, (uint16_t)address, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buff, count, 1000);
  if (res != HAL_OK)
  {
    Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] LIS2DH: Read Serial Byte Fail", FmtTimeShow());
  }
  #endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */
  FreeExclusiveLock(&i2c2Lock);
}

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : LIS2DH_GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS2DH_GetWHO_AM_I(void)
{
  uint8_t ChipID;
  /* Check Chip ID */
  ChipID = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_WHO_AM_I);
  if (ChipID == WHOAMI_LIS2DH_ACC) //LIS2DH_MEMS_I2C_ADDRESS
  {
    Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] LIS2DH: ID(0x%02X)", FmtTimeShow(), ChipID);
    return MEMS_SUCCESS;
  }
  else
  {
    Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] LIS2DH:Unknown Chipset(0x%X)", FmtTimeShow(), ChipID);
  }
  return MEMS_ERROR;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetStatusAUX
* Description    : Read the AUX status register
* Input          : Char to empty by status register buffer
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LIS2DH_GetStatusAUX(uint8_t *val)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_STATUS_AUX);

  *val = RdRegData;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetStatusAUXBIT
* Description    : Read the AUX status register BIT
* Input          : LIS2DH_STATUS_AUX_TOR,LIS2DH_STATUS_AUX_TDA 
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetStatusAUXBit(uint8_t statusBIT, uint8_t *val)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_STATUS_AUX);

  if (statusBIT == LIS2DH_STATUS_AUX_TOR)
  {
    if (RdRegData &= LIS2DH_STATUS_AUX_TOR)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_STATUS_AUX_TDA)
  {
    if (RdRegData &= LIS2DH_STATUS_AUX_TDA)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }
  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_SetODR
* Description    : Sets LIS2DH Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetODR(LIS2DH_ODR_t ov)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG1);

  RdRegData &= 0x0f;
  RdRegData |= ov << LIS2DH_ODR_BIT;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG1, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetTemperature
* Description    : Sets LIS2DH Output Temperature
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Note           : For Read Temperature by LIS2DH_OUT_TEMP_H, LIS2DH_SetADCAux and LIS2DH_SetBDU 
				   functions must be ENABLE
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetTemperature(State_t state)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TEMP_CFG_REG);

  RdRegData &= 0xBF;
  RdRegData |= state << LIS2DH_TEMP_EN;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TEMP_CFG_REG, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetADCAux
* Description    : Sets LIS2DH Output ADC
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetADCAux(State_t state)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TEMP_CFG_REG);

  RdRegData &= 0x7F;
  RdRegData |= state << LIS2DH_ADC_PD;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TEMP_CFG_REG, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetTempRaw
* Description    : Read the Temperature Values byOutput Registers LIS2DH_OUT_TEMP_H
* Input          : Buffer to empty
* Output         : Temperature Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetTempRaw(int8_t *buff)
{
  //uint8_t RdRegDataL;
  uint8_t RdRegDataH;

  //RdRegDataL = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS,LIS2DH_OUT_TEMP_L);

  RdRegDataH = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_OUT_TEMP_H);

  *buff = (int8_t)(RdRegDataH);

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_SetMode
* Description    : Sets LIS2DH Operating Mode
* Input          : Modality (LIS2DH_NORMAL, LIS2DH_LOW_POWER, LIS2DH_POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetMode(LIS2DH_Mode_t md)
{
  uint8_t RdRegData, RdRegData2;
  static uint8_t ODR_old_value;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG1);

  RdRegData2 = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4);

  if ((RdRegData & 0xF0) == 0)
    RdRegData = RdRegData | (ODR_old_value & 0xF0); //if it comes from POWERDOWN

  switch (md)
  {
  case LIS2DH_POWER_DOWN:
    ODR_old_value = RdRegData;
    RdRegData &= 0x0F;
    break;

  case LIS2DH_NORMAL:
    RdRegData &= 0xF7;
    RdRegData |= (MEMS_RESET << LIS2DH_LPEN);
    RdRegData2 &= 0xF7;
    RdRegData2 |= (MEMS_SET << LIS2DH_HR); //set HighResolution_BIT
    break;

  case LIS2DH_LOW_POWER:
    RdRegData &= 0xF7;
    RdRegData |= (MEMS_SET << LIS2DH_LPEN);
    RdRegData2 &= 0xF7;
    RdRegData2 |= (MEMS_RESET << LIS2DH_HR); //reset HighResolution_BIT
    break;

  default:
    return MEMS_ERROR;
  }

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG1, RdRegData);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4, RdRegData2);

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_SetAxis
* Description    : Enable/Disable LIS2DH Axis
* Input          : LIS2DH_X_ENABLE/DISABLE | LIS2DH_Y_ENABLE/DISABLE | LIS2DH_Z_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetAxis(LIS2DH_Axis_t axis)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG1);

  RdRegData &= 0xF8;
  RdRegData |= (0x07 & axis);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG1, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetFullScale
* Description    : Sets the LIS2DH FullScale
* Input          : LIS2DH_FULLSCALE_2/LIS2DH_FULLSCALE_4/LIS2DH_FULLSCALE_8/LIS2DH_FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetFullScale(LIS2DH_Fullscale_t fs)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4);

  RdRegData &= 0xCF;
  RdRegData |= (fs << LIS2DH_FS);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetBDU(State_t bdu)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4);

  RdRegData &= 0x7F;
  RdRegData |= (bdu << LIS2DH_BDU);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetBLE(LIS2DH_Endianess_t ble)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4);

  RdRegData &= 0xBF;
  RdRegData |= (ble << LIS2DH_BLE);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4, RdRegData);
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetSelfTest
* Description    : Set Self Test Modality
* Input          : LIS2DH_SELF_TEST_DISABLE/ST_0/ST_1
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetSelfTest(LIS2DH_SelfTest_t st)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4);

  RdRegData &= 0xF9;
  RdRegData |= (st << LIS2DH_ST);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4, RdRegData);

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_HPFClick
* Description    : Enable/Disable High Pass Filter for click
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_HPFClickEnable(State_t hpfe)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2);

  RdRegData &= 0xFB;
  RdRegData |= (hpfe << LIS2DH_HPCLICK);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_HPFAOI1
* Description    : Enable/Disable High Pass Filter for AOI on INT_1
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_HPFAOI1Enable(State_t hpfe)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2);

  RdRegData &= 0xFE;
  RdRegData |= (hpfe << LIS2DH_HPIS1);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2, RdRegData);

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_HPFAOI2
* Description    : Enable/Disable High Pass Filter for AOI on INT_2
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_HPFAOI2Enable(State_t hpfe)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2);

  RdRegData &= 0xFD;
  RdRegData |= (hpfe << LIS2DH_HPIS2);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : LIS2DH_HPM_NORMAL_MODE_RES/LIS2DH_HPM_REF_SIGNAL/
				   LIS2DH_HPM_NORMAL_MODE/LIS2DH_HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetHPFMode(LIS2DH_HPFMode_t hpm)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2);

  RdRegData &= 0x3F;
  RdRegData |= (hpm << LIS2DH_HPM);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetHPFCutOFF(LIS2DH_HPFCutOffFreq_t hpf)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2);

  RdRegData &= 0xCF;
  RdRegData |= (hpf << LIS2DH_HPCF);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetFilterDataSel
* Description    : Set Filter Data Selection bypassed or sent to FIFO OUT register
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetFilterDataSel(State_t state)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2);

  RdRegData &= 0xF7;
  RdRegData |= (state << LIS2DH_FDS);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG2, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LIS2DH_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LIS2DH_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |              
                    LIS2DH_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LIS2DH_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |              
                    LIS2DH_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LIS2DH_WTM_ON_INT1_ENABLE/DISABLE         |           
                    LIS2DH_INT1_OVERRUN_ENABLE/DISABLE  
* example        : SetInt1Pin(LIS2DH_CLICK_ON_PIN_INT1_ENABLE | LIS2DH_I1_INT1_ON_PIN_INT1_ENABLE |              
                    LIS2DH_I1_INT2_ON_PIN_INT1_DISABLE | LIS2DH_I1_DRDY1_ON_INT1_ENABLE | LIS2DH_I1_DRDY2_ON_INT1_ENABLE |
                    LIS2DH_WTM_ON_INT1_DISABLE | LIS2DH_INT1_OVERRUN_DISABLE   ) 
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetInt1Pin(LIS2DH_IntPinConf_t pinConf)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG3);

  RdRegData &= 0x00;
  RdRegData |= pinConf;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG3, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : LIS2DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE   | LIS2DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS2DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS2DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS2DH_INT_ACTIVE_HIGH/LOW
* example        : LIS2DH_SetInt2Pin(LIS2DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LIS2DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS2DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS2DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS2DH_INT_ACTIVE_HIGH/LOW)
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetInt2Pin(LIS2DH_IntPinConf_t pinConf)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG6);

  RdRegData &= 0x00;
  RdRegData |= pinConf;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG6, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetClickCFG
* Description    : Set Click Interrupt config Function
* Input          : LIS2DH_ZD_ENABLE/DISABLE | LIS2DH_ZS_ENABLE/DISABLE  | LIS2DH_YD_ENABLE/DISABLE  | 
                   LIS2DH_YS_ENABLE/DISABLE | LIS2DH_XD_ENABLE/DISABLE  | LIS2DH_XS_ENABLE/DISABLE 
* example        : LIS2DH_SetClickCFG( LIS2DH_ZD_ENABLE | LIS2DH_ZS_DISABLE | LIS2DH_YD_ENABLE | 
                               LIS2DH_YS_DISABLE | LIS2DH_XD_ENABLE | LIS2DH_XS_ENABLE)
* Note           : You MUST use all input variable in the argument, as example
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetClickCFG(uint8_t status)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CLICK_CFG);

  RdRegData &= 0xC0;
  RdRegData |= status;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CLICK_CFG, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetClickTHS
* Description    : Set Click Interrupt threshold
* Input          : Click-click Threshold value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetClickTHS(uint8_t ths)
{
  uint8_t RdRegData;

  if (ths > 127)
    return MEMS_ERROR;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CLICK_THS);

  RdRegData &= 0x80;
  RdRegData |= ths;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG6, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetClickLIMIT
* Description    : Set Click Interrupt Time Limit
* Input          : Click-click Time Limit value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetClickLIMIT(uint8_t t_limit)
{
  uint8_t RdRegData;

  if (t_limit > 127)
    return MEMS_ERROR;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TIME_LIMIT);

  RdRegData &= 0x80;
  RdRegData |= t_limit;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TIME_LIMIT, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetClickLATENCY
* Description    : Set Click Interrupt Time Latency
* Input          : Click-click Time Latency value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetClickLATENCY(uint16_t t_latency)
{
  uint8_t RdRegData;

  if (t_latency > 255)
    return MEMS_ERROR;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TIME_LATENCY);

  RdRegData &= 0x00;
  RdRegData |= t_latency;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TIME_LATENCY, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetClickWINDOW
* Description    : Set Click Interrupt Time Window
* Input          : Click-click Time Window value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetClickWINDOW(uint8_t t_window)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TIME_WINDOW);

  RdRegData &= 0x00;
  RdRegData |= t_window;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_TIME_WINDOW, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetClickResponse
* Description    : Get Click Interrupt Response by CLICK_SRC REGISTER
* Input          : char to empty by Click Response Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetClickResponse(uint8_t *res)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CLICK_SRC);

  RdRegData &= 0x7F;

  if ((RdRegData & LIS2DH_IA) == 0)
  {
    *res = LIS2DH_NO_CLICK;
    return MEMS_SUCCESS;
  }
  else
  {
    if (RdRegData & LIS2DH_DCLICK)
    {
      if (RdRegData & LIS2DH_CLICK_SIGN)
      {
        if (RdRegData & LIS2DH_CLICK_Z)
        {
          *res = LIS2DH_DCLICK_Z_N;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_Y)
        {
          *res = LIS2DH_DCLICK_Y_N;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_X)
        {
          *res = LIS2DH_DCLICK_X_N;
          return MEMS_SUCCESS;
        }
      }
      else
      {
        if (RdRegData & LIS2DH_CLICK_Z)
        {
          *res = LIS2DH_DCLICK_Z_P;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_Y)
        {
          *res = LIS2DH_DCLICK_Y_P;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_X)
        {
          *res = LIS2DH_DCLICK_X_P;
          return MEMS_SUCCESS;
        }
      }
    }
    else
    {
      if (RdRegData & LIS2DH_CLICK_SIGN)
      {
        if (RdRegData & LIS2DH_CLICK_Z)
        {
          *res = LIS2DH_SCLICK_Z_N;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_Y)
        {
          *res = LIS2DH_SCLICK_Y_N;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_X)
        {
          *res = LIS2DH_SCLICK_X_N;
          return MEMS_SUCCESS;
        }
      }
      else
      {
        if (RdRegData & LIS2DH_CLICK_Z)
        {
          *res = LIS2DH_SCLICK_Z_P;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_Y)
        {
          *res = LIS2DH_SCLICK_Y_P;
          return MEMS_SUCCESS;
        }
        if (RdRegData & LIS2DH_CLICK_X)
        {
          *res = LIS2DH_SCLICK_X_P;
          return MEMS_SUCCESS;
        }
      }
    }
  }
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_Int1LatchEnable(State_t latch)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5);

  RdRegData &= 0xF7;
  RdRegData |= latch << LIS2DH_LIR_INT1;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_ResetInt1Latch(void)
{
  AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_SRC);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetIntConfiguration
* Description    : Interrupt 1 Configuration (without LIS2DH_6D_INT)
* Input          : LIS2DH_INT1_AND/OR | LIS2DH_INT1_ZHIE_ENABLE/DISABLE | LIS2DH_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetIntConfiguration(LIS2DH_Int1Conf_t ic)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_CFG);

  RdRegData &= 0x40;
  RdRegData |= ic;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_CFG, RdRegData);

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_SetIntMode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : LIS2DH_INT_MODE_OR, LIS2DH_INT_MODE_6D_MOVEMENT, LIS2DH_INT_MODE_AND, 
				   LIS2DH_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetIntMode(LIS2DH_Int1Mode_t int_mode)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_CFG);

  RdRegData &= 0x3F; //--------------------------------------------------------------------------??????
  RdRegData |= (int_mode << LIS2DH_INT_6D);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_CFG, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetInt6D4DConfiguration
* Description    : 6D, 4D Interrupt Configuration
* Input          : LIS2DH_INT1_6D_ENABLE, LIS2DH_INT1_4D_ENABLE, LIS2DH_INT1_6D_4D_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetInt6D4DConfiguration(LIS2DH_INT_6D_4D_t ic)
{
  uint8_t RdRegData = 0;
  uint8_t RdRegData2 = 0;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_CFG);

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5);

  if (ic == LIS2DH_INT1_6D_ENABLE)
  {
    RdRegData &= 0xBF;
    RdRegData |= (MEMS_ENABLE << LIS2DH_INT_6D);
    RdRegData2 &= 0xFB;
    RdRegData2 |= (MEMS_DISABLE << LIS2DH_D4D_INT1);
  }

  if (ic == LIS2DH_INT1_4D_ENABLE)
  {
    RdRegData &= 0xBF;
    RdRegData |= (MEMS_ENABLE << LIS2DH_INT_6D);
    RdRegData2 &= 0xFB;
    RdRegData2 |= (MEMS_ENABLE << LIS2DH_D4D_INT1);
  }

  if (ic == LIS2DH_INT1_6D_4D_DISABLE)
  {
    RdRegData &= 0xBF;
    RdRegData |= (MEMS_DISABLE << LIS2DH_INT_6D);
    RdRegData2 &= 0xFB;
    RdRegData2 |= (MEMS_DISABLE << LIS2DH_D4D_INT1);
  }

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_CFG, RdRegData);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_Get6DPosition
* Description    : 6D, 4D Interrupt Position Detect
* Input          : Byte to empty by POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_Get6DPosition(uint8_t *val)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_SRC);

  RdRegData &= 0x7F;

  switch (RdRegData)
  {
  case LIS2DH_UP_SX:
    *val = LIS2DH_UP_SX;
    break;
  case LIS2DH_UP_DX:
    *val = LIS2DH_UP_DX;
    break;
  case LIS2DH_DW_SX:
    *val = LIS2DH_DW_SX;
    break;
  case LIS2DH_DW_DX:
    *val = LIS2DH_DW_DX;
    break;
  case LIS2DH_TOP:
    *val = LIS2DH_TOP;
    break;
  case LIS2DH_BOTTOM:
    *val = LIS2DH_BOTTOM;
    break;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetInt1Threshold(uint8_t ths)
{
  uint8_t RdRegData = 0;
  if (ths > 127)
    return MEMS_ERROR;

  RdRegData |= ths;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_THS, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetInt1Duration(LIS2DH_Int1Conf_t id)
{
  uint8_t RdRegData = 0;

  if (id > 127)
    return MEMS_ERROR;

  RdRegData |= id;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_DURATION, RdRegData);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LIS2DH_FIFO_DISABLE, LIS2DH_FIFO_BYPASS_MODE, LIS2DH_FIFO_MODE, 
				   LIS2DH_FIFO_STREAM_MODE, LIS2DH_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_FIFOModeEnable(LIS2DH_FifoMode_t fm)
{
  uint8_t RdRegData;

  if (fm == LIS2DH_FIFO_DISABLE)
  {
    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG);

    RdRegData &= 0x1F;
    RdRegData |= (LIS2DH_FIFO_BYPASS_MODE << LIS2DH_FM);

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG, RdRegData); //fifo mode bypass

    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5);

    RdRegData &= 0xBF;

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5, RdRegData); //fifo disable
  }

  if (fm == LIS2DH_FIFO_BYPASS_MODE)
  {
    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5);

    RdRegData &= 0xBF;
    RdRegData |= MEMS_SET << LIS2DH_FIFO_EN;

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5, RdRegData); //fifo enable

    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG);

    RdRegData &= 0x1f;
    RdRegData |= (fm << LIS2DH_FM); //fifo mode configuration

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG, RdRegData);
  }

  if (fm == LIS2DH_FIFO_MODE)
  {
    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5);

    RdRegData &= 0xBF;
    RdRegData |= MEMS_SET << LIS2DH_FIFO_EN;

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5, RdRegData); //fifo enable

    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG);

    RdRegData &= 0x1f;
    RdRegData |= (fm << LIS2DH_FM); //fifo mode configuration

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG, RdRegData);
  }

  if (fm == LIS2DH_FIFO_STREAM_MODE)
  {
    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5);

    RdRegData &= 0xBF;
    RdRegData |= MEMS_SET << LIS2DH_FIFO_EN;

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5, RdRegData); //fifo enable

    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG);

    RdRegData &= 0x1f;
    RdRegData |= (fm << LIS2DH_FM); //fifo mode configuration

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG, RdRegData);
  }

  if (fm == LIS2DH_FIFO_TRIGGER_MODE)
  {
    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5);

    RdRegData &= 0xBF;
    RdRegData |= MEMS_SET << LIS2DH_FIFO_EN;

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG5, RdRegData); //fifo enable

    RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG);

    RdRegData &= 0x1f;
    RdRegData |= (fm << LIS2DH_FM); //fifo mode configuration

    AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG, RdRegData);
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetTriggerInt
* Description    : Trigger event liked to trigger signal INT1/INT2
* Input          : LIS2DH_TRIG_INT1/LIS2DH_TRIG_INT2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetTriggerInt(LIS2DH_TrigInt_t tr)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG);

  RdRegData &= 0xDF;
  RdRegData |= (tr << LIS2DH_TR);

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG, RdRegData);

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetWaterMark(uint8_t wtm)
{
  uint8_t RdRegData;

  if (wtm > 31)
    return MEMS_ERROR;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG);

  RdRegData &= 0xE0;
  RdRegData |= wtm;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_CTRL_REG, RdRegData);

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH_GetStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetStatusReg(uint8_t *val)
{
  uint8_t RdRegData = 0;

  RdRegData |= *val;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_STATUS_REG);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetStatusBIT
* Description    : Read the status register BIT
* Input          : LIS2DH_STATUS_REG_ZYXOR, LIS2DH_STATUS_REG_ZOR, LIS2DH_STATUS_REG_YOR, LIS2DH_STATUS_REG_XOR,
                   LIS2DH_STATUS_REG_ZYXDA, LIS2DH_STATUS_REG_ZDA, LIS2DH_STATUS_REG_YDA, LIS2DH_STATUS_REG_XDA, 
				   LIS2DH_DATAREADY_BIT
				   val: Byte to be filled with the status bit	
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetStatusBit(uint8_t statusBIT, uint8_t *val)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_STATUS_REG);

  RdRegData |= *val;

  switch (statusBIT)
  {
  case LIS2DH_STATUS_REG_ZYXOR:
    if (RdRegData &= LIS2DH_STATUS_REG_ZYXOR)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  case LIS2DH_STATUS_REG_ZOR:
    if (RdRegData &= LIS2DH_STATUS_REG_ZOR)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  case LIS2DH_STATUS_REG_YOR:
    if (RdRegData &= LIS2DH_STATUS_REG_YOR)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  case LIS2DH_STATUS_REG_XOR:
    if (RdRegData &= LIS2DH_STATUS_REG_XOR)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  case LIS2DH_STATUS_REG_ZYXDA:
    if (RdRegData &= LIS2DH_STATUS_REG_ZYXDA)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  case LIS2DH_STATUS_REG_ZDA:
    if (RdRegData &= LIS2DH_STATUS_REG_ZDA)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  case LIS2DH_STATUS_REG_YDA:
    if (RdRegData &= LIS2DH_STATUS_REG_YDA)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  case LIS2DH_STATUS_REG_XDA:
    if (RdRegData &= LIS2DH_STATUS_REG_XDA)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetAccAxesRaw(AxesRaw_t *buff)
{
  uint16_t RdRegData;
  uint8_t *RdRegDataL = (uint8_t *)(&RdRegData);
  uint8_t *RdRegDataH = ((uint8_t *)(&RdRegData) + 1);

  *RdRegDataL = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_OUT_X_L);

  *RdRegDataH = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_OUT_X_H);

  buff->AXIS_X = RdRegData;

  *RdRegDataL = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_OUT_Y_L);

  *RdRegDataH = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_OUT_Y_H);

  buff->AXIS_Y = RdRegData;

  *RdRegDataL = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_OUT_Z_L);

  *RdRegDataH = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_OUT_Z_H);

  buff->AXIS_Z = RdRegData;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : Char to empty by Int1 source value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetInt1Src(uint8_t *val)
{
  uint8_t RdRegData = 0;

  RdRegData |= *val;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_SRC);
  Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] INT1_SRC bit mask value:[0x%x]", FmtTimeShow(), RdRegData);
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : statusBIT: LIS2DH_INT_SRC_IA, LIS2DH_INT_SRC_ZH, LIS2DH_INT_SRC_ZL.....
*                  val: Byte to be filled with the status bit
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetInt1SrcBit(uint8_t statusBIT, uint8_t *val)
{
  uint8_t RdRegData = 0;

  RdRegData |= *val;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_INT1_SRC);

  if (statusBIT == LIS2DH_INT1_SRC_IA)
  {
    if (RdRegData &= LIS2DH_INT1_SRC_IA)
    {
      *val = MEMS_SET;

      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_INT1_SRC_ZH)
  {
    if (RdRegData &= LIS2DH_INT1_SRC_ZH)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_INT1_SRC_ZL)
  {
    if (RdRegData &= LIS2DH_INT1_SRC_ZL)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_INT1_SRC_YH)
  {
    if (RdRegData &= LIS2DH_INT1_SRC_YH)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_INT1_SRC_YL)
  {
    if (RdRegData &= LIS2DH_INT1_SRC_YL)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }
  if (statusBIT == LIS2DH_INT1_SRC_XH)
  {
    if (RdRegData &= LIS2DH_INT1_SRC_XH)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_INT1_SRC_XL)
  {
    if (RdRegData &= LIS2DH_INT1_SRC_XL)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetFifoSourceReg(uint8_t *val)
{
  uint8_t RdRegData = 0;

  RdRegData |= *val;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_SRC_REG);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : statusBIT: LIS2DH_FIFO_SRC_WTM, LIS2DH_FIFO_SRC_OVRUN, LIS2DH_FIFO_SRC_EMPTY
*				   val: Byte to fill  with the bit value
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_GetFifoSourceBit(uint8_t statusBIT, uint8_t *val)
{
  uint8_t RdRegData = 0;

  RdRegData |= *val;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_SRC_REG);

  if (statusBIT == LIS2DH_FIFO_SRC_WTM)
  {
    if (RdRegData &= LIS2DH_FIFO_SRC_WTM)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_FIFO_SRC_OVRUN)
  {
    if (RdRegData &= LIS2DH_FIFO_SRC_OVRUN)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }

  if (statusBIT == LIS2DH_FIFO_SRC_EMPTY)
  {
    if (RdRegData &= statusBIT == LIS2DH_FIFO_SRC_EMPTY)
    {
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else
    {
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }
  }
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_GetFifoSourceFSS
* Description    : Read current number of unread samples stored in FIFO
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS2DH_GetFifoSourceFSS(uint8_t *val)
{
  uint8_t RdRegData = 0;

  RdRegData &= 0x1F;

  RdRegData |= *val;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_FIFO_SRC_REG);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2DH_SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : LIS2DH_SPI_3_WIRE, LIS2DH_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH_SetSPIInterface(LIS2DH_SPIMode_t spi)
{
  uint8_t RdRegData;

  RdRegData = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4);

  RdRegData &= 0xFE;
  RdRegData |= spi << LIS2DH_SIM;

  AccelWriteRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_CTRL_REG4, RdRegData);

  return MEMS_SUCCESS;
}

/**
  * @brief Interrupt Position Detect
  * @param LIS2DH_POSITION_6D_t direction
			LIS2DH_UP_SX       =     0x44,                  
			LIS2DH_UP_DX       =     0x42,
			LIS2DH_DW_SX      =     0x41,
			LIS2DH_DW_DX      =     0x48,
			LIS2DH_TOP           =     0x60,
			LIS2DH_BOTTOM     =     0x50,
			LIS2DH_UNKOWN    =     0xFF
  * @retval none
*/
void Gsensor_6DPosition_Handle(LIS2DH_POSITION_6D_t direction)
{
  uint8_t response;
  response = LIS2DH_Get6DPosition((uint8_t *)&gsensordirection);
  if ((response == 1) && (old_gsensordirection != gsensordirection))
  {
    switch (gsensordirection)
    {
    case LIS2DH_UP_SX:
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] APP: Position = UP_SX  [0x%x]", FmtTimeShow(), gsensordirection);
      break;
    case LIS2DH_UP_DX:
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] APP: Position = UP_DX [0x%x]", FmtTimeShow(), gsensordirection);
      break;
    case LIS2DH_DW_SX:
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] APP: Position = DW_SX [0x%x]", FmtTimeShow(), gsensordirection);
      break;
    case LIS2DH_DW_DX:
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] APP: Position = DW_DX [0x%x]", FmtTimeShow(), gsensordirection);
      break;
    case LIS2DH_TOP:
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] APP: Position = TOP [0x%x]", FmtTimeShow(), gsensordirection);
      break;
    case LIS2DH_BOTTOM:
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] APP: Position = BOTTOM [0x%x]", FmtTimeShow(), gsensordirection);
      break;
    default:
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] APP: Position = unknown [0x%x]", FmtTimeShow(), gsensordirection);
      break;
    }
    old_gsensordirection = gsensordirection;
  }

  // Print Out
  //Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn,"\r\n[%s] APP: position = [0x%x]",FmtTimeShow( ),direction);

  // Add Customer Data Start

  // Add Customer Data End
}

void GsensorInterruptRoutine(void)
{
  //uint8_t RdRegData;

  /*20171031 modify to make this file independent*/
  Gsensor_6DPosition_Handle((LIS2DH_POSITION_6D_t)gsensordirection);

  //LIS2DH_GetInt1Src(&RdRegData);

  LIS2DH_ResetInt1Latch();
}

void GsensorIntProcess(void)
{
  // if (HAL_GPIO_ReadPin(GSENSOR_INT1_PORT, GSENSOR_INT1_PIN) == GPIO_PIN_SET)
  if (GetAccIntHappenStatus() == TRUE)
  {
    Lis2dhMemsChipID = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_WHO_AM_I);
    if (Lis2dhMemsChipID == WHOAMI_LIS2DH_ACC)
    {
      Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s]Gsensor Interupt Occured", FmtTimeShow());
      GsensorInterruptRoutine();
    }

    SetAccIntHappenStatus(FALSE);
    /* Clear Acc gpio it */
    // __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
  }
}

/**
  * @brief Enable gsensor  
  * @param void
  * @retval none
*/
void GsensorStart(void)
{
  GSensorI2cInit();
}

/**
  * @brief Disable gsensor  
  * @param void
  * @retval none
*/
void GsensorStop(void)
{
  Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s] LIS2DH: Disable", FmtTimeShow());
  LIS2DH_SetMode(LIS2DH_POWER_DOWN);
}

void GsensorGetRawAxes(void)
{
  AxesRaw_t data;
  LIS2DH_GetAccAxesRaw(&data);
  Lis2dhPrintf(DbgCtl.Lis2dhDbgInfoEn, "\r\n[%s]  APP: X=%6d Y=%6d Z=%6d ", FmtTimeShow(), data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
}

/*******************************************************************************
  Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
