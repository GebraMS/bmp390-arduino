/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Sepehr Azimi
 * ________________________________________________________________________________________________________
 */
 
#include "GebraBit_BMP390.h"
#include <math.h>
	
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of BMP390
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_BMP390_Read_Reg_Data(uint8_t regAddr, uint8_t* data) {    
    uint8_t status = 1;  // Assume 1 for error, 0 for success
    uint8_t txBuf = regAddr | 0x80; // Read operation: set the 8th-bit to 1
    uint8_t rxBuf;

    // Select the BMP390 by pulling the chip select pin low
    digitalWrite(SPI_CS_Pin, LOW);

    // Transmit the register address and read the data
    SPI.transfer(txBuf);
    SPI.transfer(0xFF);
	rxBuf = SPI.transfer(0xFF);

    // Deselect the BMP390 by pulling the chip select pin high
    digitalWrite(SPI_CS_Pin, HIGH);

    *data = rxBuf;
    status = 0;  // Indicate success
    return status;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of BMP390 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_BMP390_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data)
{
    uint8_t status = 1;
    uint8_t tempData = 0;

    if (len > 8 || start_bit > 7) {
        return 1;  // Error
    }

    if (GB_BMP390_Read_Reg_Data(regAddr, &tempData) == 0) {
        uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); // Create mask to extract desired bits
        tempData &= mask; // Isolate the desired bits
        tempData >>= (start_bit - len + 1); // Align the bits to LSB
        *data = tempData;
        status = 0;  // Success
    } else {
        status = 1;  // Error
        *data = 0;
    }
    return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of BMP390 that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_BMP390_Burst_Read(uint8_t regAddr, uint8_t* data, uint16_t byteQuantity) {
    uint8_t status = 1;  // Assume 1 for error, 0 for success

    // Select the BMP390 by pulling the chip select pin low
    digitalWrite(SPI_CS_Pin, LOW);

    // Transmit the register address with read bit
    SPI.transfer(regAddr | 0x80);
	SPI.transfer(0xFF);

    // Read the data bytes
    for (uint16_t i = 0; i < byteQuantity; i++) {
        data[i] = SPI.transfer(0xFF);
    }

    // Deselect the BMP390 by pulling the chip select pin high
    digitalWrite(SPI_CS_Pin, HIGH);

    status = 0;  // Indicate success
    return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of BMP390
 * @param     data    Value that will be writen to register .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_BMP390_Write_Reg_Data(uint8_t regAddr, uint8_t data) {
    uint8_t status = 1;  // Assume 1 for error, 0 for success

    // Select the BMP390 by pulling the chip select pin low
    digitalWrite(SPI_CS_Pin, LOW);

    // Transmit the register address and data
    SPI.transfer(regAddr | 0x00); // Write operation: clear the 8th-bit
    SPI.transfer(data);

    // Deselect the BMP390 by pulling the chip select pin high
    digitalWrite(SPI_CS_Pin, HIGH);

    status = 0;  // Indicate success
    return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of BMP390 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_BMP390_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data) {
    uint8_t status = 1;
    uint8_t tempData = 0;

    if (len > 8 || start_bit > 7) {
        return 1;  // Error
    }

    if (GB_BMP390_Read_Reg_Data(regAddr, &tempData) == 0) {
        uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
        data <<= (start_bit - len + 1); // Shift data into correct position
        data &= mask; // Zero out non-important bits in data
        tempData &= ~mask; // Zero out important bits in existing byte
        tempData |= data; // Combine data with existing byte

        // Write the modified data back to the register
        status = GB_BMP390_Write_Reg_Data(regAddr, tempData);
    }
    return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of BMP390 that writing multiple data start from this address
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     byteQuantity Quantity of data that we want to write .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_BMP390_Burst_Write(uint8_t regAddr, uint8_t *data, uint16_t byteQuantity) {
    uint8_t txBuf[byteQuantity + 1]; // +1 is for the register address that is 1 byte
    txBuf[0] = regAddr | 0x00; // Write operation: set the 8th-bit to 0

    // Manually copy the data array into the txBuf
    for (uint16_t i = 0; i < byteQuantity; i++) {
        txBuf[i + 1] = data[i];
    }

    // Pull the CS pin low to select the BMP390
    digitalWrite(SPI_CS_Pin, LOW);

    // Transmit data over SPI
    for (uint16_t i = 0; i < byteQuantity + 1; i++) {
        SPI.transfer(txBuf[i]);
    }

    // Pull the CS pin high to deselect the BMP390
    digitalWrite(SPI_CS_Pin, HIGH);

    return 0; // Return 0 to indicate success
}
/*=========================================================================================================================================
 * @brief     Reset BMP390
 * @param     BMP390   BMP390 Struct RESET  variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BMP390_Soft_Reset ( GebraBit_BMP390 * BMP390 )
{
	GB_BMP390_Check_Sensor_Status( BMP390 );
	if(BMP390->SENSOR_STATUS == CMD_RDY)
	{
	 do 
	 {
		GB_BMP390_Write_Reg_Data (BMP390_CMD , BMP390_SOFT_RESET_CMD);
		delay(2);
    	GB_BMP390_Check_Error_Codition( BMP390 );
		if ( BMP390->ERROR_CONDITION != CMD_ERR ){
			BMP390->RESET = DONE;
			break;}
	 }while(1);
  }
}
/*=========================================================================================================================================
 * @brief     Get Device ID
 * @param     BMP390     BMP390 Struct DEVICE_ID variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_BMP390_Get_Device_ID(GebraBit_BMP390 * BMP390)
{
	GB_BMP390_Read_Reg_Data( BMP390_CHIP_ID,&BMP390->DEVICE_ID);
}	
/*=========================================================================================================================================
 * @brief     Get Revision ID
 * @param     BMP390     BMP390 Struct REVISION_ID variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_BMP390_Get_Revision_ID(GebraBit_BMP390 * BMP390)
{
	GB_BMP390_Read_Reg_Data( BMP390_REV_ID,&BMP390->REVISION_ID);
}	
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature Sensor
 * @param     BMP390   BMP390 Struct TEMPERATURE  variable
 * @param     temp     Determines DISABLE or ENABLE Temperature Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Temperature(GebraBit_BMP390* BMP390 ,BMP390_Ability temp)
{
	GB_BMP390_Write_Reg_Bits (BMP390_PWR_CTRL , START_MSB_BIT_AT_1, BIT_LENGTH_1 , temp);
  BMP390->TEMPERATURE = temp ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Pressure Sensor
 * @param     BMP390    BMP390 Struct PRESSURE  variable
 * @param     press     Determines DISABLE or ENABLE Pressure Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_BMP390_Pressure(GebraBit_BMP390 * BMP390 , BMP390_Ability press)
{
	GB_BMP390_Write_Reg_Bits (BMP390_PWR_CTRL , START_MSB_BIT_AT_0, BIT_LENGTH_1 , press);
  BMP390->PRESSURE = press ; 
}
/*=========================================================================================================================================
 * @brief     Set Sensor Output Sample Rate that controls  data output rate
 * @param     BMP390   BMP390 struct  OUTPUT_DATA_RATE variable
 * @param     rate  Values are According to BMP390_Output_Data_Rate Enum
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Output_Sample_Rate (GebraBit_BMP390 * BMP390 , BMP390_Output_Data_Rate rate)
{
	GB_BMP390_Write_Reg_Bits(BMP390_ODR, START_MSB_BIT_AT_4, BIT_LENGTH_5,rate );
	BMP390->OUTPUT_DATA_RATE = rate  ;
}
/*=========================================================================================================================================
 * @brief     Set IIR Filter Coefficient
 * @param     BMP390  BMP390 Struct IIR_FILTER variable
 * @param     filter     Values are According to BMP390_IIR_Filter_Coefficient Enum
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_BMP390_IIR_Filter_Coefficient (GebraBit_BMP390 * BMP390 , BMP390_IIR_Filter_Coefficient filter)
{
	GB_BMP390_Write_Reg_Bits(BMP390_CONFIG, START_MSB_BIT_AT_3, BIT_LENGTH_3,filter );
	BMP390->IIR_FILTER = filter  ;
}
/*=========================================================================================================================================
 * @brief     Check Sensor Status
 * @param     BMP390   BMP390 Struct SENSOR_STATUS  variable
 * @return    Nothing
 ========================================================================================================================================*/

void GB_BMP390_Check_Sensor_Status(GebraBit_BMP390 * BMP390 )
{
	uint8_t tempData;
	GB_BMP390_Read_Reg_Data( BMP390_STATUS,&tempData);
	BMP390->SENSOR_STATUS = static_cast<BMP390_Sensor_Status>(tempData);
}
/*=========================================================================================================================================
 * @brief     Check Error Codition
 * @param     BMP390   BMP390 Struct ERROR_CONDITION  variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BMP390_Check_Error_Codition(GebraBit_BMP390 * BMP390 ) 
{
	uint8_t tempData;
	GB_BMP390_Read_Reg_Data( BMP390_ERR_REG,&tempData);
	BMP390->ERROR_CONDITION = static_cast<BMP390_Error_Condition>(tempData);
}
/*=========================================================================================================================================
 * @brief     Check FIFO Full Interrupt
 * @param     BMP390   BMP390 Struct INTERRUPT_STATUS  variable
 * @return    Nothing
 ========================================================================================================================================*/ 

void GB_BMP390_Check_FIFO_Full_Interrupt_(GebraBit_BMP390 * BMP390 )
{
	uint8_t stat = 0 ;
	GB_BMP390_Read_Reg_Data( BMP390_INT_STATUS,&stat);
	//GB_BMP390_Read_Reg_Bits(BMP390_INT_STATUS, START_MSB_BIT_AT_1, BIT_LENGTH_1,&BMP390->INTERRUPT_STATUS );
	BMP390->INTERRUPT_STATUS= (BMP390_Interrupt_Status)(stat&0x02);
}
/*=========================================================================================================================================
 * @brief     Check FIFO Full Interrupt
 * @param     BMP390   BMP390 Struct INTERRUPT_STATUS  variable
 * @return    Nothing
 ========================================================================================================================================*/ 

void GB_BMP390_Check_Data_Ready_Interrupt(GebraBit_BMP390 * BMP390 )
{
	uint8_t stat = 0 ;
	GB_BMP390_Read_Reg_Data( BMP390_INT_STATUS,&stat);
	//GB_BMP390_Read_Reg_Bits(BMP390_INT_STATUS, START_MSB_BIT_AT_1, BIT_LENGTH_1,&BMP390->INTERRUPT_STATUS );
	BMP390->INTERRUPT_STATUS= (BMP390_Interrupt_Status)(stat&0x08);
}
/*=========================================================================================================================================
 * @brief     SET Temperature OverSampling
 * @param     BMP390   BMP390 Struct TEMPRATURE_OVERSAMPLING  variable
 * @param     temp_over   Values are According to BMP390_Sensor_Oversampling Enum
 * @return    Nothing
 ========================================================================================================================================*/  
 void GB_BMP390_Temperature_OverSampling(GebraBit_BMP390* BMP390 ,BMP390_Sensor_Oversampling temp_over)
{
	GB_BMP390_Write_Reg_Bits (BMP390_OSR , START_MSB_BIT_AT_5, BIT_LENGTH_3 , temp_over);
	BMP390->TEMPRATURE_OVERSAMPLING = temp_over ;
}
/*=========================================================================================================================================
 * @brief     SET Pressure OverSampling
 * @param     BMP390   BMP390 Struct PRESSURE_OVERSAMPLING  variable
 * @param     press_over   Values are According to BMP390_Sensor_Oversampling Enum
 * @return    Nothing
 ========================================================================================================================================*/ 
 void GB_BMP390_Pressure_OverSampling(GebraBit_BMP390* BMP390 ,BMP390_Sensor_Oversampling press_over)
{
	GB_BMP390_Write_Reg_Bits (BMP390_OSR , START_MSB_BIT_AT_2, BIT_LENGTH_3 , press_over);
	BMP390->PRESSURE_OVERSAMPLING = press_over ;
}
/*=========================================================================================================================================
 * @brief     Set BMP390  Power Mode
 * @param     BMP390   BMP390 Struct POWER_MODE  variable
 * @param     pmode    Values are According to BMP390_Power_Mode Enum
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Power_Mode(GebraBit_BMP390* BMP390 ,BMP390_Power_Mode pmode)
{
	GB_BMP390_Write_Reg_Bits (BMP390_PWR_CTRL , START_MSB_BIT_AT_5, BIT_LENGTH_2 , pmode);
	BMP390->POWER_MODE = pmode ;
}

/*=========================================================================================================================================
 * @brief     Configure hardware interrupt pin (INT) 
 * @param     BMP390  BMP390 struct INT_PIN_LEVEL , INT_PIN_TYPE and INT_PIN_LATCH  variables
 * @param     level   ACTIVE_HIGH or  ACTIVE_LOW 
 * @param     type    PUSH_PULL   or  OPEN_DRAIN
 * @param     latch   NOT_LATCH   or   LATCH
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BMP390_Set_INT_Pin(GebraBit_BMP390 * BMP390 , BMP390_INT_Level level ,BMP390_INT_Type type , BMP390_Latch_Type latch )
{
  GB_BMP390_Write_Reg_Bits( BMP390_INT_CTRL, START_MSB_BIT_AT_1, BIT_LENGTH_1 , level);
	GB_BMP390_Write_Reg_Bits( BMP390_INT_CTRL, START_MSB_BIT_AT_0, BIT_LENGTH_1 , type);
	GB_BMP390_Write_Reg_Bits( BMP390_INT_CTRL, START_MSB_BIT_AT_2, BIT_LENGTH_1 , latch);
	BMP390->INT_PIN_LEVEL = level ; 
	BMP390->INT_PIN_TYPE  = type  ;
	BMP390->INT_PIN_LATCH = latch ;
}
/*=========================================================================================================================================
 * @brief     Set Select Data_Output type
 * @param     BMP390   BMP390 Struct OUTPUT_DATA  variable
 * @param     data_sel    Values are According to BMP390_Data_Select Enum
 * @return    Nothing
 ========================================================================================================================================*/ 
/*
M403Z 
*/
void GB_BMP390_Data_Output_Select(GebraBit_BMP390 * BMP390 , BMP390_Data_Select data_sel) 
{ 
	GB_BMP390_Write_Reg_Bits (BMP390_FIFO_CONFIG_2 , START_MSB_BIT_AT_4, BIT_LENGTH_2,  data_sel);
  BMP390->OUTPUT_DATA = data_sel ;  
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Data Ready Interrupt
 * @param     BMP390   BMP390 Struct DATA_READY_INT  variable
 * @param     data_ready_int    Determines Data Ready Interrupt Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Data_Ready_Interrupt(GebraBit_BMP390 * BMP390 , BMP390_Ability data_ready_int)
{
	GB_BMP390_Write_Reg_Bits(BMP390_INT_CTRL, START_MSB_BIT_AT_6, BIT_LENGTH_1 , data_ready_int);
	BMP390->DATA_READY_INT = data_ready_int  ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Data FIFO 
 * @param     BMP390   BMP390 Struct FIFO  variable
 * @param     fifo    Determines FIFO  Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_FIFO(GebraBit_BMP390 * BMP390  , BMP390_Ability fifo)
{
	GB_BMP390_Write_Reg_Bits (BMP390_FIFO_CONFIG_1 , START_MSB_BIT_AT_0, BIT_LENGTH_1 , fifo);
	BMP390->FIFO = fifo ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE FIFO Overflow Interrupt
 * @param     BMP390   BMP390 Struct FIFO_FULL_INT  variable
 * @param     fifo_full_int    Determines  FIFO Full Interrupt Disable or Enable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_FIFO_Full_Interrupt(GebraBit_BMP390 * BMP390 , BMP390_Ability fifo_full_int)
{
	GB_BMP390_Write_Reg_Bits(BMP390_INT_CTRL, START_MSB_BIT_AT_4, BIT_LENGTH_1 , fifo_full_int);
	BMP390->FIFO_FULL_INT = fifo_full_int  ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE SensorTime to be written on FIFO
 * @param     BMP390  BMP390 struct TIME_TO_FIFO  variable  
 * @param     time_fifo Determines SensorTime write on fifo or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Write_SensorTime_FIFO(GebraBit_BMP390 * BMP390 , BMP390_Ability time_fifo )
{
   GB_BMP390_Write_Reg_Bits (BMP390_FIFO_CONFIG_1, START_MSB_BIT_AT_2, BIT_LENGTH_1,time_fifo); 
	 BMP390->TIME_TO_FIFO = time_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Pressure to be written on FIFO
 * @param     BMP390  BMP390 struct PRESS_TO_FIFO  variable  
 * @param     time_fifo Determines Pressure write on fifo or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Write_Pressure_FIFO(GebraBit_BMP390 * BMP390 , BMP390_Ability press_fifo )
{
   GB_BMP390_Write_Reg_Bits (BMP390_FIFO_CONFIG_1, START_MSB_BIT_AT_3, BIT_LENGTH_1,press_fifo); ///*7 to make b111 to enable xyz
	 BMP390->PRESS_TO_FIFO = press_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature to be written on FIFO
 * @param     BMP390  BMP390 struct TEMP_TO_FIFO  variable  
 * @param     time_fifo Determines Temperature write on fifo or not
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_BMP390_Write_Temperature_FIFO(GebraBit_BMP390 * BMP390 , BMP390_Ability temp_fifo )
{
   GB_BMP390_Write_Reg_Bits (BMP390_FIFO_CONFIG_1, START_MSB_BIT_AT_4, BIT_LENGTH_1,temp_fifo); 
	 BMP390->TEMP_TO_FIFO = temp_fifo ;
}
/*=========================================================================================================================================
 * @brief     Set FIFO MODE
 * @param     BMP390  BMP390 struct FIFO_MODE  variable 
 * @param     fifo_mode     Determines FIFO MODE BYPASS ,  STREAM_TO_FIFO , STOP_ON_FULL_FIFO_SNAPSHOT
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_FIFO_Mode(GebraBit_BMP390 * BMP390 , BMP390_FIFO_Mode fifo_mode )
{
  GB_BMP390_Write_Reg_Bits (BMP390_FIFO_CONFIG_1,START_MSB_BIT_AT_1, BIT_LENGTH_1, fifo_mode); 
  BMP390->FIFO_MODE = fifo_mode;
}

/*=========================================================================================================================================
 * @brief     Set FIFO DownSampling  
 * @param     BMP390     BMP390 struct  FIFO_SUBSAMPLING variable
 * @param     dwnsmple   SET devider
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_FIFO_DownSampling(GebraBit_BMP390 * BMP390,uint8_t dwnsmple)
{
	GB_BMP390_Write_Reg_Bits(BMP390_FIFO_CONFIG_2,START_MSB_BIT_AT_2, BIT_LENGTH_3 , dwnsmple );
	BMP390->FIFO_SUBSAMPLING = pow(2,dwnsmple) ; 
}
/*=========================================================================================================================================
 * @brief     SET FIFO WATERMARK 
 * @param     BMP390     BMP390 struct  FIFO_WATERMARK variable
 * @param     watermark     Determines FIFO WATERMARK Enable or not
 * @param     wm            Determines FIFO WATERMARK Value
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_BMP390_FIFO_WATERMARK (GebraBit_BMP390 * BMP390,BMP390_Ability watermark , uint16_t wm)
{	
  GB_BMP390_Write_Reg_Bits (BMP390_INT_CTRL, START_MSB_BIT_AT_3, BIT_LENGTH_1 , watermark);
	GB_BMP390_Write_Reg_Data (BMP390_FIFO_WTM_0,(uint8_t) (wm & 0xff));
	GB_BMP390_Write_Reg_Bits (BMP390_FIFO_WTM_1,START_MSB_BIT_AT_0, BIT_LENGTH_1 ,(uint8_t) (wm>> 8));			
	BMP390->FIFO_WATERMARK = watermark ;  
}
/*=========================================================================================================================================
 * @brief     Get FIFO Length
 * @param     BMP390   BMP390 struct  FIFO_LENGTH variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_GET_FIFO_Length (GebraBit_BMP390 * BMP390 ) 
{
	uint8_t count_h , count_l;
  GB_BMP390_Read_Reg_Data( BMP390_FIFO_LENGTH_1, &count_h); 
	GB_BMP390_Read_Reg_Data( BMP390_FIFO_LENGTH_0, &count_l );
	BMP390->FIFO_LENGTH = (uint16_t)((count_h << 8) | count_l);////13_Bit
}
/*=========================================================================================================================================
 * @brief     Set FIFO reset.
 * @param     BMP390   BMP390 struct  SENSOR_STATUS & ERROR_CONDITION variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_FIFO_Flush(GebraBit_BMP390 * BMP390 ) 
{
	GB_BMP390_Check_Sensor_Status( BMP390 );
	if(BMP390->SENSOR_STATUS == CMD_RDY)
	{
	 do 
	 {
		GB_BMP390_Write_Reg_Data (BMP390_CMD , BMP390_FIFO_FLUSH_CMD);
		delay(2);
    GB_BMP390_Check_Error_Codition( BMP390 );
		if ( BMP390->ERROR_CONDITION != CMD_ERR )
			break;
	 }while(1);
  }
}
/*=========================================================================================================================================
// * @brief     Read Data Directly from FIFO
// * @param     BMP390  BMP390 struct FIFO_DATA variable
// * @param     qty    Determine how many Data Byte to read
// * @return    Nothing
// ========================================================================================================================================*/ 
void GB_BMP390_Read_FIFO(GebraBit_BMP390 * BMP390 , uint16_t qty)  
{
  GB_BMP390_Burst_Read( BMP390_FIFO_DATA,BMP390->FIFO_DATA, qty);
}

/*=========================================================================================================================================
 * @brief     Configure FIFO
 * @param     BMP390         BMP390 Struct FIFO variable
 * @param     fifo           Configure BMP390 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_FIFO_Configuration ( GebraBit_BMP390 * BMP390 , BMP390_FIFO_Ability fifo  )  
{
	BMP390->FIFO_DATA_BUFFER_SIZE = 504 ;         				      
  BMP390->BYTE_QTY_IN_ONE_FIFO_PACKET = 7 ;                  
  BMP390->TOTAL_FIFO_PACKET = BMP390->FIFO_DATA_BUFFER_SIZE/BMP390->BYTE_QTY_IN_ONE_FIFO_PACKET ;   
	if( fifo==FIFO_ENABLE )  
	{
		BMP390->FIFO = Enable  ; 
		GB_BMP390_FIFO_Flush(BMP390);
		GB_BMP390_Write_SensorTime_FIFO ( BMP390 , Disable );
	  GB_BMP390_Write_Pressure_FIFO( BMP390 , Enable );
		GB_BMP390_Write_Temperature_FIFO ( BMP390 , Enable );
		GB_BMP390_FIFO_Mode ( BMP390 , STOP_ON_FULL_FIFO_SNAPSHOT );
		GB_BMP390_FIFO_DownSampling(BMP390,1);
		GB_BMP390_FIFO_WATERMARK (BMP390,Disable ,0);
		GB_BMP390_FIFO_Full_Interrupt(BMP390 ,Enable) ;
		GB_BMP390_Data_Ready_Interrupt( BMP390 ,Disable ) ;
		GB_BMP390_FIFO(BMP390  , Enable);
	}
	else if ( fifo == FIFO_DISABLE )
	{
		BMP390->FIFO = Disable  ;
		GB_BMP390_FIFO_Full_Interrupt(BMP390 ,Disable) ;
		GB_BMP390_Data_Ready_Interrupt( BMP390 ,Enable ) ;
		GB_BMP390_FIFO_WATERMARK (BMP390,Disable ,0);
		GB_BMP390_Write_SensorTime_FIFO ( BMP390 , Disable );
	  GB_BMP390_Write_Pressure_FIFO( BMP390 , Disable );
		GB_BMP390_Write_Temperature_FIFO ( BMP390 , Disable );
		GB_BMP390_FIFO(BMP390  , Disable);
		GB_BMP390_FIFO_Flush(BMP390);
	}
}
/*=========================================================================================================================================
 * @brief     Set BMP390 Power Management
 * @param     pmode        Determines BMP390 Accelerometer Power Mode in NORMAL_MODE or FORCED_MODE or SLEEP_MODE
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Set_Power_Management(GebraBit_BMP390 * BMP390 , BMP390_Power_Mode pmode) 
{	
	
 GB_BMP390_Temperature(BMP390 , Enable );
 GB_BMP390_Pressure(BMP390 , Enable );
 GB_BMP390_Power_Mode(BMP390 ,pmode);
 delay(1);
}
/*=========================================================================================================================================
 * @brief     Calculate Calibration Coefficients
 * @param     BMP390   BMP390 struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Calculate_Calibration_Coefficients(GebraBit_BMP390 * BMP390)
{
 int8_t nvm_par_t3 , nvm_par_p3 , nvm_par_p4 , nvm_par_p7 , nvm_par_p8 , nvm_par_p10 , nvm_par_p11;
 uint16_t nvm_par_t1 , nvm_par_t2 , nvm_par_p5 , nvm_par_p6 ;
 int16_t nvm_par_p1 , nvm_par_p2 , nvm_par_p9 ;
 long double floating_factor ;
 GB_BMP390_Burst_Read( BMP390_CALIB_DATA,  BMP390->CALIBRATION_DATA ,  CALIBRATION_DATA_BUFFER_SIZE);
 nvm_par_t1  = ((uint16_t)BMP390->CALIBRATION_DATA[1]<<8)|((uint16_t)BMP390->CALIBRATION_DATA[0]);	
 nvm_par_t2  = ((uint16_t)BMP390->CALIBRATION_DATA[3]<<8)|((uint16_t)BMP390->CALIBRATION_DATA[2]);
 nvm_par_t3  =  BMP390->CALIBRATION_DATA[4];
 nvm_par_p1  = ((int16_t)BMP390->CALIBRATION_DATA[6]<<8)|((int16_t)BMP390->CALIBRATION_DATA[5]);	
 nvm_par_p2  = ((int16_t)BMP390->CALIBRATION_DATA[8]<<8)|((int16_t)BMP390->CALIBRATION_DATA[7]);	
 nvm_par_p3  =  BMP390->CALIBRATION_DATA[9];
 nvm_par_p4  =  BMP390->CALIBRATION_DATA[10];
 nvm_par_p5   = ((uint16_t)BMP390->CALIBRATION_DATA[12]<<8)|((uint16_t)BMP390->CALIBRATION_DATA[11]);	
 nvm_par_p6  = ((uint16_t)BMP390->CALIBRATION_DATA[14]<<8)|((uint16_t)BMP390->CALIBRATION_DATA[13]);		
 nvm_par_p7  =  BMP390->CALIBRATION_DATA[15];	
 nvm_par_p8  =  BMP390->CALIBRATION_DATA[16];
 nvm_par_p9  = ((int16_t)BMP390->CALIBRATION_DATA[18]<<8)|((int16_t)BMP390->CALIBRATION_DATA[17]);
 nvm_par_p10 =  BMP390->CALIBRATION_DATA[19];	
 nvm_par_p11 =  BMP390->CALIBRATION_DATA[20];	
 floating_factor = 0.00390625f;
 BMP390->PAR_T1  = ( double)nvm_par_t1/floating_factor;
 floating_factor = 1073741824.0f;
 BMP390->PAR_T2  = ( double)nvm_par_t2/floating_factor;
 floating_factor = 281474976710656.0f;
 BMP390->PAR_T3  = ( double)nvm_par_t3/floating_factor;		
 floating_factor = 1048576.0f;
 BMP390->PAR_P1  = ( double)nvm_par_p1/floating_factor-0.015625f;
 floating_factor = 536870912.0f;
 BMP390->PAR_P2  = ( double)nvm_par_p2/floating_factor-0.000030517578125;
 floating_factor = 4294967296.0f;
 BMP390->PAR_P3  = ( double)nvm_par_p3/floating_factor;
 floating_factor = 137438953472.0f;
 BMP390->PAR_P4  = ( double)nvm_par_p4/floating_factor;
 floating_factor = 0.125f;
 BMP390->PAR_P5  = ( double)nvm_par_p5/floating_factor;
 floating_factor = 64.0f;
 BMP390->PAR_P6  = ( double)nvm_par_p6/floating_factor;
 floating_factor = 256.0f;
 BMP390->PAR_P7  = ( double)nvm_par_p7/floating_factor;
 floating_factor = 32768.0f;
 BMP390->PAR_P8  = ( double)nvm_par_p8/floating_factor;
 floating_factor = 281474976710656.0f;
 BMP390->PAR_P9  = ( double)nvm_par_p9/floating_factor;
 floating_factor = 281474976710656.0f;
 BMP390->PAR_P10 = ( double)nvm_par_p10/floating_factor;
 floating_factor = 36893488147419103232.0f;
 BMP390->PAR_P11 = ( double)nvm_par_p11/floating_factor;

}
/*=========================================================================================================================================
 * @brief     Initialize BMP390
 * @param     BMP390     initialize BMP390 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_initialize( GebraBit_BMP390 * BMP390 )
{
  delay(3);
  GB_BMP390_Get_Device_ID(BMP390);
	GB_BMP390_Get_Revision_ID(BMP390);
	GB_BMP390_Soft_Reset(BMP390);
	GB_BMP390_Calculate_Calibration_Coefficients(BMP390);
	GB_BMP390_Set_Power_Management( BMP390 , NORMAL_MODE );
	GB_BMP390_FIFO_Configuration ( BMP390 ,FIFO_DISABLE ) ;
	GB_BMP390_Set_INT_Pin( BMP390 , ACTIVE_LOW  , OPEN_DRAIN  ,  LATCH );//NOT_LATCH
	//GB_BMP390_Data_Ready_Interrupt( BMP390 ,Enable ) ;
}
/*=========================================================================================================================================
 * @brief     Configure BMP390
 * @param     BMP390  Configure BMP390  
 * @param     fifo           Configure BMP390 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Configuration(GebraBit_BMP390 * BMP390, BMP390_FIFO_Ability fifo) 
{
	GB_BMP390_Output_Sample_Rate(BMP390,ODR_50_HZ_20_mS );
	GB_BMP390_Pressure_OverSampling(BMP390 , X4_OVERSAMPLING);
	GB_BMP390_Temperature_OverSampling(BMP390 , X1_NO_OVERSAMPLING);
	GB_BMP390_Data_Output_Select( BMP390 ,  FILTERED_DATA);
	GB_BMP390_IIR_Filter_Coefficient ( BMP390 ,  FILTER_COEFFICIENT_15);
	GB_BMP390_FIFO_Configuration ( BMP390 ,fifo ) ;
	delay(20);	
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature And Pressure from Register 
 * @param     BMP390  store Raw Data Of Temprature in GebraBit_BMP390 Staruct REGISTER_RAW_TEMPERATURE & REGISTER_RAW_PRESSURE
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Get_Register_Raw_Pressure_Temperature(GebraBit_BMP390 * BMP390 )
{
  GB_BMP390_Burst_Read( BMP390_PRESSURE_DATA_0,BMP390->REGISTER_RAW_DATA_BUFFER, REGISTER_RAW_DATA_BYTE_QTY);
	BMP390->REGISTER_RAW_PRESSURE    = ((int32_t)BMP390->REGISTER_RAW_DATA_BUFFER[2]<<16)  | ((int32_t)BMP390->REGISTER_RAW_DATA_BUFFER[1]<<8) | ((int32_t)BMP390->REGISTER_RAW_DATA_BUFFER[0])  ;
  BMP390->REGISTER_RAW_TEMPERATURE = ((int32_t)BMP390->REGISTER_RAW_DATA_BUFFER[5]<<16)  | ((int32_t)BMP390->REGISTER_RAW_DATA_BUFFER[4]<<8) | ((int32_t)BMP390->REGISTER_RAW_DATA_BUFFER[3])  ;
}
/*=========================================================================================================================================
 * @brief     Calculate Compensated Temperature Base on Datasheet Formula 
 * @param     BMP390      BMP390 struct coefficient Parameter
 * @param     raw_temp    Raw Temperature Value
 * @param     valid_temp  Compensated Temperature
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Calculate_Compensated_Temperature(GebraBit_BMP390 * BMP390 , int32_t raw_temp , double * valid_temp )	 
{
	//int32_t raw_temp = BMP390->REGISTER_RAW_TEMPERATURE;
	double cal_part1;
	double cal_part2;
	cal_part1 = (double)(raw_temp - BMP390->PAR_T1);
	cal_part2 = (double)(cal_part1 * BMP390->PAR_T2);
	*valid_temp = cal_part2 + ((cal_part1 * cal_part1) * BMP390->PAR_T3);
	//BMP390->VALID_TEMPERATURE = cal_part2 + ((cal_part1 * cal_part1) * BMP390->PAR_T3);
}
/*=========================================================================================================================================
 * @brief     Calculate Compensated Pressure Base on Datasheet Formula 
 * @param     BMP390      BMP390 struct coefficient Parameter
 * @param     raw_temp    Raw Temperature Value
 * @param     valid_temp  Compensated Temperature
 * @param     valid_press  Compensated Pressure
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Calculate_Compensated_Pressure(GebraBit_BMP390 * BMP390 , int32_t raw_press , double valid_temp ,double * valid_press ) 	
{
  //int32_t raw_press = BMP390->REGISTER_RAW_PRESSURE;
	double cal_part1;
	double cal_part2;
	double cal_part3;
	double cal_part4;
	double cal_part5;
	double cal_part6;
	cal_part1 = BMP390->PAR_P6 * valid_temp;
	cal_part2 = BMP390->PAR_P7 * valid_temp*valid_temp;
	cal_part3 = BMP390->PAR_P8 * valid_temp*valid_temp*valid_temp;
	cal_part5  = BMP390->PAR_P5 + cal_part1 + cal_part2 + cal_part3;
	cal_part1 = BMP390->PAR_P2 * valid_temp;
	cal_part2 = BMP390->PAR_P3 * valid_temp*valid_temp;
	cal_part3 = BMP390->PAR_P4 * valid_temp*valid_temp*valid_temp;
	cal_part6  = raw_press * (BMP390->PAR_P1 + cal_part1 + cal_part2 + cal_part3);
	cal_part1 = (double)raw_press * (double)raw_press;
	cal_part2 = BMP390->PAR_P9 + BMP390->PAR_P10 * valid_temp;
	cal_part3 = cal_part1 * cal_part2;
	cal_part4 = cal_part3 + ((double)raw_press * (double)raw_press * (double)raw_press) * BMP390->PAR_P11;
	* valid_press = (cal_part5 + cal_part6 + cal_part4)/100;//pa to mili bar(with /100)
}

/*=========================================================================================================================================
 * @brief     Devide Data from FIFO to COMPENSATED Pressure And COMPENSATED Temperature & FIFO HEADER
 * @param     BMP390   BMP390 struct COMPENSATED_FIFO_TEMPERATURE and COMPENSATED_FIFO_PRESSURE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_FIFO_Data_Partition_Pressure_Temperature(GebraBit_BMP390 * BMP390)
{
 	uint16_t i,offset=0;
	uint16_t j = 0;
	int32_t fifo_temperature =0;
	int32_t fifo_pressure =0;
	 for ( i = 0 ; i < BMP390->TOTAL_FIFO_PACKET ;  )	
		{
			BMP390->FIFO_HEADER[i] = BMP390->FIFO_DATA[offset];
			if(BMP390->FIFO_HEADER[i]==FIFO_TEMP_PRESS_FRAME)
			{
				fifo_temperature = ((int32_t)BMP390->FIFO_DATA[offset+3]<<16)  | ((int32_t)BMP390->FIFO_DATA[offset+2]<<8) | ((int32_t)BMP390->FIFO_DATA[offset+1])  ;
				GB_BMP390_Calculate_Compensated_Temperature( BMP390 , fifo_temperature , &BMP390->COMPENSATED_FIFO_TEMPERATURE[i] );
				offset += 3; 
			 	fifo_pressure    = ((int32_t)BMP390->FIFO_DATA[offset+3]<<16)  | ((int32_t)BMP390->FIFO_DATA[offset+2]<<8) | ((int32_t)BMP390->FIFO_DATA[offset+1])  ;
				GB_BMP390_Calculate_Compensated_Pressure( BMP390 , fifo_pressure , BMP390->COMPENSATED_FIFO_TEMPERATURE[i] , &BMP390->COMPENSATED_FIFO_PRESSURE[i] );
				BMP390->FIFO_ALTITUDE[i] =((1 - pow((BMP390->COMPENSATED_FIFO_PRESSURE[i]*100) / SEA_LEVEL_PRESSURE, 1/5.257)) / 0.0000225577);
				offset += 4;
        i++;    
			}
			else
				offset += 2;
	  }
}
/*=========================================================================================================================================
 * @brief     Convert Pressuer To Altitude
 * @param     BMP390   BMP390 struct ALTITUDE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BMP390_Altitude(GebraBit_BMP390 * BMP390)
{
    double altitude;

    //BMP390->ALTITUDE = ((1 - pow((BMP390->COMPENSATED_PRESSURE*100) / SEA_LEVEL_PRESSURE, 1/5.257)) / 0.0000225577);  
}
/*=========================================================================================================================================
 * @brief     Determine Grtting Data From FIFO Or Register
 * @param     get_data   According to BMP390_Get_DATA Enum 
 * @param     BMP390    BMP390 struct GET_DATA  Variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BMP390_Get_Data(GebraBit_BMP390 * BMP390 , BMP390_Get_DATA get_data)
{
 if( (get_data == FROM_REGISTER)&&(BMP390->FIFO == Disable) )
 {
	  GB_BMP390_Check_Data_Ready_Interrupt(BMP390);
		if ( BMP390->INTERRUPT_STATUS == DATA_READY_INTERRUPT )
		{
	   BMP390->GET_DATA = FROM_REGISTER ;
		 GB_BMP390_Get_Register_Raw_Pressure_Temperature(BMP390) ;	
		 GB_BMP390_Check_Data_Ready_Interrupt(BMP390);
		 GB_BMP390_Calculate_Compensated_Temperature(BMP390 , BMP390->REGISTER_RAW_TEMPERATURE , &BMP390->COMPENSATED_TEMPERATURE)	;
		 GB_BMP390_Calculate_Compensated_Pressure(BMP390 , BMP390->REGISTER_RAW_PRESSURE , BMP390->COMPENSATED_TEMPERATURE , &BMP390->COMPENSATED_PRESSURE)	;	 
	   GB_BMP390_Altitude(BMP390);
		}
 } 
 else if ((get_data == FROM_FIFO)&&(BMP390->FIFO == Enable)) 
 {
		GB_BMP390_Check_FIFO_Full_Interrupt_(BMP390);
		if ( BMP390->INTERRUPT_STATUS == FIFO_FULL_INTERRUPT )
		{
			BMP390->GET_DATA = FROM_FIFO ;
		  GB_BMP390_GET_FIFO_Length(BMP390);
			GB_BMP390_Read_FIFO(BMP390 , BMP390->FIFO_LENGTH);
			GB_BMP390_FIFO_Data_Partition_Pressure_Temperature(BMP390); 
			GB_BMP390_Check_FIFO_Full_Interrupt_(BMP390);
			GB_BMP390_FIFO_Flush(BMP390);
		}
 }	 
}

/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/



//GB_BMP390_Read_Reg_Data ( BMP390_CHIP_ID, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_REV_ID, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_ERR_REG, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_STATUS, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_EVENT, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_INT_STATUS, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_FIFO_LENGTH_0, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_FIFO_LENGTH_1, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_FIFO_WTM_0, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_FIFO_WTM_1, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_FIFO_CONFIG_1, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_FIFO_CONFIG_2, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_INT_CTRL, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Write_Reg_Bits (BMP390_INT_CTRL , START_MSB_BIT_AT_5, BIT_LENGTH_1 , 0);
//GB_BMP390_Read_Reg_Data ( BMP390_INT_CTRL, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_IF_CONF, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_PWR_CTRL, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Write_Reg_Bits (BMP390_PWR_CTRL , START_MSB_BIT_AT_5, BIT_LENGTH_2 , 3);
//GB_BMP390_Read_Reg_Data ( BMP390_PWR_CTRL, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_OSR, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_ODR, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_CONFIG, &BMP390_Module.REGISTER_CACHE);
//GB_BMP390_Read_Reg_Data ( BMP390_INT_STATUS, &BMP390_Module.REGISTER_CACHE);

















