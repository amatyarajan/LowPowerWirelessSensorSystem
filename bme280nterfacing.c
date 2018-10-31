/*
 * bme280nterfacing.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Rajan
 */
#include "bme280Interfacing.h"
#include "driverlib.h"
#include <string.h>
#include <stdio.h>


s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BME280_INIT_VALUE;
    array[BME280_INIT_VALUE] = reg_addr;
    uint8_t i;
    uint16_t rtnval, debugdump;

    for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
        array[stringpos + BME280_DATA_INDEX] = *(reg_data + stringpos);
    }
    /*
    * Please take the below function as your reference for
    * write the data using I2C communication
    * "IERROR = I2C_WRITE_STRING(DEV_ADDR, array, cnt+1)"
    * add your I2C write function here
    * iError is an return value of I2C read function
    * Please select your valid return value
    * In the driver SUCCESS defined as 0
    * and FAILURE defined as -1
    * Note :
    * This is a full duplex operation,
    * The first read data is discarded, for that extra write operation
    * have to be initiated. For that cnt+1 operation done in the I2C write string function
    * For more information please refer data sheet SPI communication:
    */

    while(UCB1STATW&0x0010){};         // wait for I2C ready
    UCB1CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
    UCB1TBCNT = cnt+1;                     // generate stop condition after this many bytes
    UCB1CTLW0 &= ~0x0001;              // enable eUSCI module
    UCB1I2CSA = dev_addr;              // I2CCSA[6:0] is slave address
    UCB1CTLW0 = ((UCB1CTLW0&~0x0004)   // clear bit2 (UCTXSTP) for no transmit stop condition
                                       // set bit1 (UCTXSTT) for transmit start condition
                  | 0x0012);           // set bit4 (UCTR) for transmit mode
    while((UCB1IFG&0x0002) == 0){};    // wait for slave address sent

    for(i=0; i<cnt; i++) {
        UCB1TXBUF = array[i]&0xFF;         // TXBUF[7:0] is data
        while((UCB1IFG&0x0002) == 0){      // wait for data sent
            if(UCB1IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
                debugdump = UCB1IFG;           // snapshot flag register for calling program
                I2C_Init();                    // reset to known state
                return iError=-1;
            }
        }
    }
    UCB1TXBUF = array[i]&0xFF;         // TXBUF[7:0] is last data
    while(UCB1STATW&0x0010){           // wait for I2C idle
      if(UCB1IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
        debugdump = UCB1IFG;           // snapshot flag register for calling program
        I2C_Init();                    // reset to known state
        return iError=-1;
      }
    }
    return iError=0;
}



/* \Brief: The function is used as I2C bus read
*  \Return : Status of the I2C read
*  \param dev_addr : The device address of the sensor
*  \param reg_addr : Address of the first register, will data is going to be read
*  \param reg_data : This data read from the sensor, which is hold in an array
*  \param cnt : The no of data byte of to be read
*/
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
   s32 iError = BME280_INIT_VALUE;
   uint8_t i;
   uint16_t rtnval, debugdump;
   u8 array[I2C_BUFFER_LEN] = {BME280_INIT_VALUE};
   u8 stringpos = BME280_INIT_VALUE;
   array[BME280_INIT_VALUE] = reg_addr;
   /* Please take the below function as your reference
    * for read the data using I2C communication
    * add your I2C read function here.
    * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
    * iError is an return value of write function
    * Please select your valid return value
    * In the driver SUCCESS defined as 0
    * and FAILURE defined as -1
    */

   // set pointer to register address
   while(UCB1STATW&0x0010){};         // wait for I2C ready
   UCB1CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
   UCB1TBCNT = 1;                     // generate stop condition after this many bytes
   UCB1CTLW0 &= ~0x0001;              // enable eUSCI module
   UCB1I2CSA = dev_addr;                 // I2CCSA[6:0] is slave address
   UCB1CTLW0 = ((UCB1CTLW0&~0x0004)   // clear bit2 (UCTXSTP) for no transmit stop condition
                                      // set bit1 (UCTXSTT) for transmit start condition
                 | 0x0012);           // set bit4 (UCTR) for transmit mode
   while(UCB1CTLW0&0x0002){};         // wait for slave address sent
   UCB1TXBUF = reg_addr&0xFF;            // TXBUF[7:0] is data
   while(UCB1STATW&0x0010){           // wait for I2C idle
     if(UCB1IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
       debugdump = UCB1IFG;           // snapshot flag register for calling program
       I2C_Init();                    // reset to known state
       return iError=-1;
     }
   }

   // receive bytes from registers on BME280 device

   while(UCB1STATW&0x0010){};         // wait for I2C ready
   UCB1CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
   UCB1TBCNT = cnt;                     // generate stop condition after this many bytes
   UCB1CTLW0 &= ~0x0001;              // enable eUSCI module
   UCB1I2CSA = dev_addr;                 // I2CCSA[6:0] is slave address
   UCB1CTLW0 = ((UCB1CTLW0&~0x0014)   // clear bit4 (UCTR) for receive mode
                                      // clear bit2 (UCTXSTP) for no transmit stop condition
                 | 0x0002);           // set bit1 (UCTXSTT) for transmit start condition
   for(i=0; i<cnt; i++) {
       while((UCB1IFG&0x0001) == 0){      // wait for complete character received
         if(UCB1IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
           I2C_Init();                    // reset to known state
           return 0xFFFF;
         }
       }
       *reg_data++= UCB1RXBUF&0xFF;            // get the reply
   }

   return (s8)iError;
}

/*  Brief : The delay routine
*  \param : delay in ms
*/

/* delay */
void BME280_delay_msek(u32 msek)
{
 Delay1ms(10);
 }


/*--------------------------------------------------------------------------*
*   The following function is used to map the I2C bus read, write, delay and
*   device address with global structure bme280
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
*  By using bme280 the following structure parameter can be accessed
*  Bus write function pointer: BME280_WR_FUNC_PTR
*  Bus read function pointer: BME280_RD_FUNC_PTR
*  Delay function pointer: delay_msec
*  I2C address: dev_addr
*--------------------------------------------------------------------------*/
   bme280.bus_write = BME280_I2C_bus_write;
   bme280.bus_read = BME280_I2C_bus_read;
   bme280.dev_addr = BME280_I2C_ADDRESS1;
   bme280.delay_msec = BME280_delay_msek;

   return BME280_INIT_VALUE;
}


void bme280Init()
{
    I2C_Init();  // initialize eUSCI
    memset(resultsBuffer,0x00, 16);

        /* The variable used to assign the standby time*/
        v_stand_by_time_u8 = BME280_INIT_VALUE;
        /* The variable used to read uncompensated temperature*/
         v_data_uncomp_temp_s32 = BME280_INIT_VALUE;
        /* The variable used to read uncompensated pressure*/
         v_data_uncomp_pres_s32 = BME280_INIT_VALUE;
        /* The variable used to read uncompensated pressure*/
         v_data_uncomp_hum_s32 = BME280_INIT_VALUE;
        /* The variable used to read compensated temperature*/
         u32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
        /* The variable used to read compensated pressure*/
         u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
        /* The variable used to read compensated humidity*/
         u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};

        /* result of communication results*/
        s32  com_rslt = ERROR;

     /*********************** START INITIALIZATION ************************/
      /*    Based on the user need configure I2C or SPI interface.
      * It is example code to explain how to use the bme280 API*/
        I2C_routine();
        /*SPI_routine();*/
    /*--------------------------------------------------------------------------*
     *  This function used to assign the value/reference of
     *  the following parameters
     *  I2C address
     *  Bus Write
     *  Bus read
     *  Chip id
    *-------------------------------------------------------------------------*/
        com_rslt = bme280_init(&bme280);

        /*  For initialization it is required to set the mode of
         *  the sensor as "NORMAL"
         *  data acquisition/read/write is possible in this mode
         *  by using the below API able to set the power mode as NORMAL*/
        /* Set the power mode as NORMAL*/
        com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
        /*  For reading the pressure, humidity and temperature data it is required to
         *  set the OSS setting of humidity, pressure and temperature
         * The "BME280_CTRLHUM_REG_OSRSH" register sets the humidity
         * data acquisition options of the device.
         * changes to this registers only become effective after a write operation to
         * "BME280_CTRLMEAS_REG" register.
         * In the code automated reading and writing of "BME280_CTRLHUM_REG_OSRSH"
         * register first set the "BME280_CTRLHUM_REG_OSRSH" and then read and write
         * the "BME280_CTRLMEAS_REG" register in the function*/
        com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

        /* set the pressure oversampling*/
        com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
        /* set the temperature oversampling*/
        com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
    /*--------------------------------------------------------------------------*/
    /*------------------------------------------------------------------------*
    ************************* START GET and SET FUNCTIONS DATA ****************
    *---------------------------------------------------------------------------*/
        /* This API used to Write the standby time of the sensor input
         *  value have to be given
         *  Normal mode comprises an automated perpetual cycling between an (active)
         *  Measurement period and an (inactive) standby period.
         *  The standby time is determined by the contents of the register t_sb.
         *  Standby time can be set using BME280_STANDBYTIME_125_MS.
         *  Usage Hint : bme280_set_standbydur(BME280_STANDBYTIME_125_MS)*/

      com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    //    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1000_MS);

        /* This API used to read back the written value of standby time*/
        com_rslt += bme280_get_standby_durn(&v_stand_by_time_u8);
    /*-----------------------------------------------------------------*
    ************************* END GET and SET FUNCTIONS ****************
    *------------------------------------------------------------------*/
        // get registers

        BME280_I2C_bus_read(p_bme280->dev_addr, 0xF2, regData, 12);
    /************************* END INITIALIZATION *************************/
        /*-----------------------------------------------------------------------*
        ************************* START DE-INITIALIZATION ***********************
        *-------------------------------------------------------------------------*/
            /*  For de-initialization it is required to set the mode of
             *  the sensor as "SLEEP"
             *  the device reaches the lowest power consumption only
             *  In SLEEP mode no measurements are performed
             *  All registers are accessible
             *  by using the below API able to set the power mode as SLEEP*/
             /* Set the power mode as SLEEP*/
            com_rslt += bme280_set_power_mode(BME280_SLEEP_MODE);
        /*---------------------------------------------------------------------*
        ************************* END DE-INITIALIZATION **********************
        *---------------------------------------------------------------------*/

}






void getSensorValues(char *temperature,char *pressure,char *humidity, char *light, char *voltage, char *light2, char *voltage2, char *voltage3, char *light3)
{
    com_rslt= bme280_set_power_mode(0x01); // set power mode to forced to generate reading
    Delay1ms(10);
    if(timerDelay==0)
    {

    // get registers

    BME280_I2C_bus_read(p_bme280->dev_addr, 0xF2, regData, 12);

    /*------------------------------------------------------------------*
    ************ START READ UNCOMPENSATED PRESSURE, TEMPERATURE
    AND HUMIDITY DATA ********
    *---------------------------------------------------------------------*/
        /* API is used to read the uncompensated temperature*/
        com_rslt += bme280_read_uncomp_temperature(&v_data_uncomp_temp_s32);

        /* API is used to read the uncompensated pressure*/
        com_rslt += bme280_read_uncomp_pressure(&v_data_uncomp_pres_s32);

        /* API is used to read the uncompensated humidity*/
        com_rslt += bme280_read_uncomp_humidity(&v_data_uncomp_hum_s32);

        /* API is used to read the uncompensated temperature,pressure
        and humidity data */
//            com_rslt += bme280_read_uncomp_pressure_temperature_humidity(
//            &v_data_uncomp_temp_s32, &v_data_uncomp_pres_s32, &v_data_uncomp_hum_s32);
    /*--------------------------------------------------------------------*
    ************ END READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
    *-------------------------------------------------------------------------*/

    /*------------------------------------------------------------------*

    ************ START READ COMPENSATED PRESSURE, TEMPERATURE
    AND HUMIDITY DATA ********
    *---------------------------------------------------------------------*/
        /* API is used to compute the compensated temperature*/
        v_comp_temp_s32[0] = bme280_compensate_temperature_int32(
                v_data_uncomp_temp_s32);

        /* API is used to compute the compensated pressure*/
        v_comp_press_u32[0] = bme280_compensate_pressure_int32(
                v_data_uncomp_pres_s32);

        /* API is used to compute the compensated humidity*/
        v_comp_humidity_u32[0] = bme280_compensate_humidity_int32(
                v_data_uncomp_hum_s32);

        /* API is used to read the compensated temperature, humidity and pressure*/
//            com_rslt += bme280_read_pressure_temperature_humidity(
//            &v_comp_press_u32[1], &v_comp_temp_s32[1],  &v_comp_humidity_u32[1]);
    /*--------------------------------------------------------------------*
    ************ END READ COMPENSATED PRESSURE, TEMPERATURE AND HUMIDITY ********
    *-------------------------------------------------------------------------*/
       // printf("temp=%d, pressure=%d, humidity=%d\n",v_comp_temp_s32[0],v_comp_press_u32[0],v_comp_humidity_u32[0]);
        //printf("Float Testing %f", 16.780);
        uint32_t t = v_comp_temp_s32[0];
        uint32_t p = v_comp_press_u32[0];
        uint32_t h = v_comp_humidity_u32[0];
    /*-----------------------------------------------------------------------*/
        pressureCompensation = v_comp_press_u32[0] + 2340;

        temperature_bme280 = (float) v_comp_temp_s32[0] * 0.01;
        pressure_bme280 = (float) pressureCompensation *0.01 * 0.02953;
        humidity_bme280 = (float) v_comp_humidity_u32[0] /1024;

        sprintf(temperature,"%0.2f",temperature_bme280);
        sprintf(pressure,"%0.2f",pressure_bme280);
        sprintf(humidity,"%0.2f",humidity_bme280);


        MAP_ADC14_toggleConversionTrigger();
                    // flag=0;


       sprintf(light,"%0.2f",current);
       sprintf(voltage,"%0.2f",voltage_read);
       sprintf(light2,"%0.2f",current2);
       sprintf(voltage2,"%0.2f",voltage_read2);
       sprintf(voltage3,"%0.2f",voltage_read3);
       sprintf(light3,"%0.2f",current3);



    timerDelay++;
    }
}








/* ADC Interrupt Handler. This handler is called whenever there is a conversion
 * that is finished for ADC_MEM0.
 */
void ADC14_IRQHandler(void)
{
    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if (ADC_INT8 & status)
    {
        MAP_ADC14_getMultiSequenceResult(resultsBuffer);
      //  flag=1;

        /* Getting values for current and voltage from the result buffer*/
        curADCResult0= (int16_t)resultsBuffer[0]-8192;
        current= (float)(((curADCResult0*3.3)/8192)*1000);

        curADCResult1= (int16_t)resultsBuffer[4]-8192;
        current2= (float)(((curADCResult1*3.3)/8192)*1000);

        current3= (float)(current - current2);


        voltage_read= (resultsBuffer[6]*3.3)/16384;
        voltage_read2= (resultsBuffer[7]*3.3)/16384;
        voltage_read3= (resultsBuffer[8]*3.3)/16384;

        /*
      if (voltage_read3>= 2.0)
          {
            battery_volt= 4.2;
           }
       else if(voltage_read3>1.75 && <2.0)
          {
             battery_volt= 3.7;
           }
       else if (voltage_read>1.26 && <1.0)
           {
             battery_volt= 3.0;
            }
       else
       {
          battery_volt=2.7;
        }
       */
       }
}








