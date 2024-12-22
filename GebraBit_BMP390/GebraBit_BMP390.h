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
#ifndef	_BMP390__H_
#define	_BMP390__H_
#include "arduino.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "SPI.h"
#include "string.h"
#include "math.h"
#define SPI_CS_Pin 10
/************************************************
 *         USER BANK 0 REGISTER MAP             *
 ***********************************************/ 
#define BMP390_CHIP_ID_VALUE                  (0x60)
#define BMP390_CHIP_ID                        (0x00)
#define BMP390_REV_ID                         (0x01)
#define BMP390_ERR_REG                        (0x02)
#define BMP390_STATUS                         (0x03)
#define BMP390_PRESSURE_DATA_0                (0x04)
#define BMP390_PRESSURE_DATA_1                (0x05)
#define BMP390_PRESSURE_DATA_2                (0x06)
#define BMP390_TEMPERATURE_DATA_3             (0x07)
#define BMP390_TEMPERATURE_DATA_4             (0x08)
#define BMP390_TEMPERATURE_DATA_5             (0x09)
#define BMP390_SENSORTIME_0                   (0x0C)
#define BMP390_SENSORTIME_1                   (0x0D)
#define BMP390_SENSORTIME_2                   (0x0E)
#define BMP390_EVENT                          (0x10)
#define BMP390_INT_STATUS                     (0x11)
#define BMP390_FIFO_LENGTH_0                  (0x12)
#define BMP390_FIFO_LENGTH_1                  (0x13)
#define BMP390_FIFO_DATA                      (0x14)
#define BMP390_FIFO_WTM_0                     (0x15)
#define BMP390_FIFO_WTM_1                     (0x16)
#define BMP390_FIFO_CONFIG_1                  (0x17)
#define BMP390_FIFO_CONFIG_2                  (0x18)
#define BMP390_INT_CTRL                       (0x19)
#define BMP390_IF_CONF                        (0x1A)
#define BMP390_PWR_CTRL                       (0x1B)
#define BMP390_OSR                            (0X1C)
#define BMP390_ODR                            (0x1D)
#define BMP390_CONFIG                         (0x1F)
#define BMP390_CALIB_DATA                     (0x31)
#define BMP390_CMD                            (0x7E)
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 

/************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define BMP390_SOFT_RESET_CMD                     (0xB6)
#define BMP390_FIFO_FLUSH_CMD                     (0xB0)
#define CALIBRATION_DATA_BUFFER_SIZE               21
#define PRESSURE_BYTE_QTY          						     3
#define TEMPERATURE_BYTE_QTY           						 3
#define FIFO_BUFFER_SIZE         				           508 
#define TOTAL_PACKET                					     72
#define REGISTER_RAW_DATA_BYTE_QTY                 6
#define PACKET_QTY_IN_FULL_FIFO       						 73    
#define SEA_LEVEL_PRESSURE									     	101325
//#define BYTE_QTY_IN_ONE_FIFO_PACKET              7  
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/****************************************************
 *    			Values For ERR_REG Register  					  *
 ****************************************************/ 
typedef enum Error_Condition
{  
 FATAL_ERR = 1 ,                         
 CMD_ERR   = 2 ,                          
 CONF_ERR  = 4                          					 
}BMP390_Error_Condition;
/******************************************************
 * 					  	Values For STATUS Register			  	  *
 ******************************************************/ 
typedef enum Sensor_Status
{  
 CMD_RDY    = 0x10 ,
 DRDY_PRESS = 0x20 ,
 DRDY_TEMP  = 0x40     														
}BMP390_Sensor_Status;
/**************************************************
 *         Values For INT_STATUS Register         *
 **************************************************/ 
typedef enum Interrupt_Status
{  
 FIFO_WATERMARK_INTERRUPT = 0x01 ,              
 FIFO_FULL_INTERRUPT      = 0x02 ,                 
 DATA_READY_INTERRUPT     = 0x08                  
}BMP390_Interrupt_Status;
/******************************************************
 *     Values For FS_SEL in GYRO_CONFIG  Register     *
 ******************************************************/ 
typedef enum Data_Select
{ 
  UNFILTERED_DATA = 0 ,	                               
	FILTERED_DATA                     								      							
}BMP390_Data_Select;
/*************************************************
 *       Values For FIFO_CONFIG_2 Register       *
 **************************************************/ 
typedef enum FIFO_Mode 
{  
	STREAM_TO_FIFO = 0 ,                              
	STOP_ON_FULL_FIFO_SNAPSHOT = 1                    
}BMP390_FIFO_Mode ;

/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum Ability 
{  
	Disable = 0     ,                      
	Enable     
}BMP390_Ability;        
/**************************************************
 *      			alues For PWR_CTRL Register         *
 **************************************************/ 
typedef enum Power_Mode
{
	SLEEP_MODE  = 0,        							
	FORCED_MODE = 1, 											                   
	NORMAL_MODE = 3
} BMP390_Power_Mode;
/**************************************************************
 *        				 Values For Oversampling			    		      *
 **************************************************************/ 
typedef enum Pressure_Oversampling
{
	 X1_NO_OVERSAMPLING = 0 ,										
	 X2_OVERSAMPLING    = 1 , 									
	 X4_OVERSAMPLING    = 2 ,										  
	 X8_OVERSAMPLING    = 3 ,                    
	 X16_OVERSAMPLING   = 4 ,											
	 X32_OVERSAMPLING   = 5		                  
} BMP390_Sensor_Oversampling;
/**************************************************************
 *        						Values For ODR Register 				        *
 **************************************************************/ 
typedef enum Output_Data_Rate
{
	 ODR_200_HZ_5_mS         = 0 ,                  
	 ODR_100_HZ_10_mS        = 1 ,									
	 ODR_50_HZ_20_mS         = 2 ,									
	 ODR_25_HZ_40_mS         = 3 ,
	 ODR_12P5_HZ_80_mS       = 4 ,                  
	 ODR_6P25_HZ_160_mS      = 5 ,									
	 ODR_3P1_HZ_320_mS       = 6 ,									
	 ODR_1P5_HZ_640_mS       = 7 ,
	 ODR_0P78_HZ_1280_mS     = 8 ,                  
	 ODR_0P39_HZ_2560_mS     = 9 ,									
	 ODR_0P2_HZ_5120_mS      = 10 ,									
	 ODR_0P1_HZ_10240_mS     = 11 ,
	 ODR_0P05_HZ_20480_mS    = 12 ,                  
	 ODR_0P02_HZ_40960_mS    = 13 ,									
	 ODR_0P01_HZ_81920_mS    = 14 ,									
	 ODR_0P006_HZ_163840_mS  = 15 ,	
	 ODR_0P003_HZ_327680_mS  = 16 ,									
	 ODR_0P0015_HZ_655360_mS = 17
} BMP390_Output_Data_Rate;
/**************************************************************
 *        				Values For CONFIG Register        	 			  *
 **************************************************************/ 
typedef enum IIR_Filter_Coefficient
{
	 FILTER_BYPASS_MODE     = 0 ,                  
	 FILTER_COEFFICIENT_1   = 1 ,									
	 FILTER_COEFFICIENT_3   = 2 ,									
	 FILTER_COEFFICIENT_7   = 3 ,
	 FILTER_COEFFICIENT_15  = 4 ,                  
	 FILTER_COEFFICIENT_31  = 5 ,									
	 FILTER_COEFFICIENT_63  = 6 ,									
	 FILTER_COEFFICIENT_127 = 7 
} BMP390_IIR_Filter_Coefficient;
/**************************************************************
 *           				Values For FIFO Header    		  	 			  *
 **************************************************************/
typedef enum FIFO_Header
{
	 FIFO_EMPTY_FRAME     	 = 0x80 ,                  
	 FIFO_CONFIG_CHANGE   	 = 0x48 ,									
	 FIFO_ERROR_FRAME   		 = 0x44 ,									
	 FIFO_TIME_FRAME   		 	 = 0xA0 ,
	 FIFO_PRESS_FRAME  		 	 = 0x84 ,                  
	 FIFO_TEMP_FRAME  			 = 0x90 ,									
	 FIFO_TEMP_PRESS_FRAME   = 0x94 
} BMP390_FIFO_Header;

/*************************************************
 *         Values For Data Preparation           *
 **************************************************/ 
typedef enum Preparation
{  
	IS_NOT_Ready = 0     ,                      
	IS_Ready     
}BMP390_Preparation;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	DONE     = 0     ,                      
	FAILED   = 1    
}BMP390_Reset_Status;
/**************************************************
 *       Values For Disable And Enable FIFO       *
 **************************************************/ 
typedef enum FIFO_Ability
{  
	FIFO_DISABLE = 0     ,                      
	FIFO_ENABLE     
} BMP390_FIFO_Ability;

/**************************************************
 * Values For Methode of getting data from sensor *
 **************************************************/ 
typedef enum Get_DATA
{  
	FROM_REGISTER = 0     ,                      
	FROM_FIFO     
} BMP390_Get_DATA; 
/*******************************************************
 *     Values For INT_LEVEL in INT_PIN_CFG Register    *
 *******************************************************/ 
typedef enum int_level
{  
	ACTIVE_LOW = 0 ,                      
	ACTIVE_HIGH     
} BMP390_INT_Level; 
/*******************************************************
 *     Values For INT_OPEN in INT_PIN_CFG Register     *
 *******************************************************/ 
typedef enum int_type
{  
	PUSH_PULL = 0 ,                      
	OPEN_DRAIN     
}BMP390_INT_Type; 
/*******************************************************
 *  Values For LATCH_INT_EN in INT_PIN_CFG Register    *
 *******************************************************/ 
typedef enum latch_type
{  
	NOT_LATCH = 0 ,                            
	LATCH     														
} BMP390_Latch_Type; 
 /*************************************************
 *  Defining BMP390 Register & Data As Struct   *
 **************************************************/
typedef	struct BMP390
{
	  uint8_t                       	REGISTER_CACHE;
	  BMP390_Get_DATA             		GET_DATA;
	  BMP390_Reset_Status         		RESET;
	  uint8_t                       	DEVICE_ID;
		uint8_t                       	REVISION_ID;
	  BMP390_Sensor_Status  					SENSOR_STATUS;
	  BMP390_Error_Condition					ERROR_CONDITION;
		BMP390_Power_Mode 				  	  POWER_MODE;
		BMP390_Ability                  PRESSURE;
	  BMP390_Sensor_Oversampling			PRESSURE_OVERSAMPLING;
		BMP390_Ability                  TEMPERATURE;
		BMP390_Sensor_Oversampling			TEMPRATURE_OVERSAMPLING;
	  BMP390_Output_Data_Rate         OUTPUT_DATA_RATE;
	  BMP390_Data_Select              OUTPUT_DATA;
	  BMP390_IIR_Filter_Coefficient   IIR_FILTER;
	  BMP390_Interrupt_Status					INTERRUPT_STATUS;
	  BMP390_Ability 							    DATA_READY_INT;	  
		BMP390_INT_Level                INT_PIN_LEVEL;
  	BMP390_INT_Type                 INT_PIN_TYPE;
	  BMP390_Latch_Type               INT_PIN_LATCH;		
	  BMP390_Ability        					FIFO;
		BMP390_FIFO_Mode					  		FIFO_MODE;
	  BMP390_Ability              		TEMP_TO_FIFO;
		BMP390_Ability									PRESS_TO_FIFO;
		BMP390_Ability              		TIME_TO_FIFO;
    uint16_t												FIFO_DATA_BUFFER_SIZE;         				      
    uint8_t													BYTE_QTY_IN_ONE_FIFO_PACKET;                  
    uint8_t													TOTAL_FIFO_PACKET;                					
		BMP390_Ability									FIFO_WATERMARK;
	  uint8_t                         FIFO_SUBSAMPLING;
		uint16_t                      	FIFO_LENGTH ;
		BMP390_Ability 							    FIFO_FULL_INT;
		uint8_t 												CALIBRATION_DATA[CALIBRATION_DATA_BUFFER_SIZE];
		double 													PAR_T1;
		double 													PAR_T2;
		double 													PAR_T3;
		double 													PAR_P1;
		double 													PAR_P2;
		double 													PAR_P3;
		double 													PAR_P4;
		double 													PAR_P5;
		double 													PAR_P6;
		double 													PAR_P7;
		double 													PAR_P8;
		double 													PAR_P9;
		double 													PAR_P10;
		double 													PAR_P11;
		uint8_t 												REGISTER_RAW_DATA_BUFFER[REGISTER_RAW_DATA_BYTE_QTY];
		int32_t 											  REGISTER_RAW_PRESSURE;
		int32_t 												REGISTER_RAW_TEMPERATURE;
		double 													COMPENSATED_TEMPERATURE;
	  double 													COMPENSATED_PRESSURE;
	  //double 													ALTITUDE;
		uint8_t 												FIFO_DATA[FIFO_BUFFER_SIZE];
		BMP390_FIFO_Header              FIFO_HEADER[TOTAL_PACKET];
		double												  COMPENSATED_FIFO_TEMPERATURE[TOTAL_PACKET];
		double												  COMPENSATED_FIFO_PRESSURE[TOTAL_PACKET];
		double												  FIFO_ALTITUDE[TOTAL_PACKET];

}GebraBit_BMP390;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *Declare Read&Write BMP390 Register Values Functions *
 ********************************************************/
extern	uint8_t	GB_BMP390_Read_Reg_Data ( uint8_t regAddr,uint8_t* data);
extern	uint8_t GB_BMP390_Read_Reg_Bits (uint8_t regAddr,uint8_t start_bit, uint8_t len, uint8_t* data);
extern	uint8_t GB_BMP390_Burst_Read(uint8_t regAddr,uint8_t *data, uint16_t byteQuantity);
extern	uint8_t GB_BMP390_Write_Reg_Data(uint8_t regAddr, uint8_t data);
extern	uint8_t	GB_BMP390_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data);
extern	uint8_t GB_BMP390_Burst_Write		( uint8_t regAddr,uint8_t *data, 	uint16_t byteQuantity);
/********************************************************
 *       Declare BMP390 Configuration Functions       *
 ********************************************************/
extern void GB_BMP390_Soft_Reset ( GebraBit_BMP390 * BMP390 );
extern void	GB_BMP390_Get_Device_ID(GebraBit_BMP390 * BMP390);
extern void	GB_BMP390_Get_Revision_ID(GebraBit_BMP390 * BMP390);
extern void GB_BMP390_Temperature(GebraBit_BMP390* BMP390 ,BMP390_Ability temp);
extern void GB_BMP390_Pressure(GebraBit_BMP390 * BMP390 , BMP390_Ability press);
extern void GB_BMP390_Output_Sample_Rate (GebraBit_BMP390 * BMP390 , BMP390_Output_Data_Rate rate);
extern void GB_BMP390_IIR_Filter_Coefficient (GebraBit_BMP390 * BMP390 , BMP390_IIR_Filter_Coefficient filter) ;
extern void GB_BMP390_Check_Sensor_Status(GebraBit_BMP390 * BMP390 );
extern void GB_BMP390_Check_Error_Codition(GebraBit_BMP390 * BMP390 );
extern void GB_BMP390_Check_FIFO_Full_Interrupt_(GebraBit_BMP390 * BMP390 );
extern void GB_BMP390_Check_Data_Ready_Interrupt(GebraBit_BMP390 * BMP390 );
extern void GB_BMP390_Temperature_OverSampling(GebraBit_BMP390* BMP390 ,BMP390_Sensor_Oversampling temp_over) ;
extern void GB_BMP390_Pressure_OverSampling(GebraBit_BMP390* BMP390 ,BMP390_Sensor_Oversampling press_over);
extern void GB_BMP390_Power_Mode(GebraBit_BMP390* BMP390 ,BMP390_Power_Mode pmode);
extern void GB_BMP390_Set_INT_Pin(GebraBit_BMP390 * BMP390 , BMP390_INT_Level level ,BMP390_INT_Type type , BMP390_Latch_Type latch ) ;
extern void GB_BMP390_Data_Output_Select(GebraBit_BMP390 * BMP390 , BMP390_Data_Select data_sel) ;
extern void GB_BMP390_Data_Ready_Interrupt(GebraBit_BMP390 * BMP390 , BMP390_Ability data_ready_int);
extern void GB_BMP390_FIFO(GebraBit_BMP390 * BMP390  , BMP390_Ability fifo) ;
extern void GB_BMP390_FIFO_Full_Interrupt(GebraBit_BMP390 * BMP390 , BMP390_Ability fifo_full_int) ;
extern void GB_BMP390_Write_SensorTime_FIFO(GebraBit_BMP390 * BMP390 , BMP390_Ability time_fifo );
extern void GB_BMP390_Write_Pressure_FIFO(GebraBit_BMP390 * BMP390 , BMP390_Ability press_fifo );
extern void GB_BMP390_Write_Temperature_FIFO(GebraBit_BMP390 * BMP390 , BMP390_Ability temp_fifo );
extern void GB_BMP390_FIFO_Mode(GebraBit_BMP390 * BMP390 , BMP390_FIFO_Mode fifo_mode );
extern void GB_BMP390_FIFO_DownSampling(GebraBit_BMP390 * BMP390,uint8_t dwnsmple);
extern void GB_BMP390_FIFO_WATERMARK (GebraBit_BMP390 * BMP390,BMP390_Ability watermark , uint16_t wm);
extern void GB_BMP390_GET_FIFO_Length (GebraBit_BMP390 * BMP390 ) ;
extern void GB_BMP390_FIFO_Flush(GebraBit_BMP390 * BMP390 );
extern void GB_BMP390_Read_FIFO(GebraBit_BMP390 * BMP390 , uint16_t qty);
extern void GB_BMP390_FIFO_Configuration ( GebraBit_BMP390 * BMP390 , BMP390_FIFO_Ability fifo  );
/********************************************************
 *          Declare BMP390 DATA Functions               *
 ********************************************************/
extern void GB_BMP390_Get_Register_Raw_Pressure_Temperature(GebraBit_BMP390 * BMP390 )  ;
extern void GB_BMP390_Calculate_Compensated_Temperature(GebraBit_BMP390 * BMP390 , int32_t raw_temp , double * valid_temp )	;
extern void GB_BMP390_Calculate_Compensated_Pressure(GebraBit_BMP390 * BMP390 , int32_t raw_press , double valid_temp ,double * valid_press );
extern void GB_BMP390_FIFO_Data_Partition_Pressure_Temperature(GebraBit_BMP390 * BMP390);
extern void GB_BMP390_Altitude(GebraBit_BMP390 * BMP390);
extern void GB_BMP390_Get_Data(GebraBit_BMP390 * BMP390 , BMP390_Get_DATA get_data);
/********************************************************
 *          Declare BMP390 HIGH LEVEL Functions       *
 ********************************************************/
extern void GB_BMP390_Set_Power_Management(GebraBit_BMP390 * BMP390 , BMP390_Power_Mode pmode) ;
extern void GB_BMP390_initialize( GebraBit_BMP390 * BMP390 );
extern void GB_BMP390_Configuration(GebraBit_BMP390 * BMP390, BMP390_FIFO_Ability fifo);

#endif
