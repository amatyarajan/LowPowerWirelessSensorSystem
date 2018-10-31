
//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//This Thesis 11 is created to set CH_PD high and low
//
//****************************************************************************




#include "msp.h"
#include "driverlib.h"
#include "clk.h"
#include "gpio.h"
#include <string.h>
#include <stdbool.h>
#include "bme280Interfacing.h"
#include "i2c.h"
#include "uart.h"
#include "ST7735.h"
#include "rtc.h"
#include <stdio.h>
#include <stdint.h>




uint8_t U2RXData;
void getDateAndTime();
void Delay1ms();
uint8_t  writecommand(uint8_t c);


typedef enum {
    screen1=0,
    screen2
}_screen;

_screen screen= screen1;

bool timeset= 0;
int twoSecondToggle = 0;
bool timeBlinker = 0;
bool spreadsheet =0;
int sensorCount=0;
int screenOff=0;
int wakeup=0;
int less_battery=0;
int connection_entered=0;
int gotdata=0;
int batteryVoltage= 0;

RTC_C_Calendar newTime;//to store updated time
RTC_C_Calendar currentTime;//to store updated time

char receivingBuffer[2500];
char timeBuffer[32];
int bufferposition = 0;
char displayBuff [20];

char at_commands [100];
char get_command[300];

//char SSID [] = "SM-G950U66E";
//char password [] = "6163041458";

char SSID []= "Himalayan Panthers";
char password [] = "Aayogorkhali@2016";


char deviceId []= "v5007A5FF339097A";
//char deviceId [] = "v076640F39375C65";

#define ST7735_DIS0N   0x29
#define ST7735_DISPOFF  0x28


SENSOR bme280_sensor;


volatile char temperature[7];
volatile char pressure[7];
volatile char humidity[7];
volatile char current_solar[7];
volatile char voltage[7];
volatile char current_ltc[7];
volatile char voltage2[7];
volatile char voltage3[7];
volatile char current_battery[7];




typedef enum  {CheckAdc, ESPReset, ConnectionSetUp, SetESPMode, ConnectToRouter, TCPConnect,LCDinit,UpdateData,END,Display,ConnectSpreadSheet} states;
states CurrentState;


void DelayWait10ms(uint32_t n){
  Delay1ms(n*10);
}



void display(int row, int col, char *str, int front, int back, int size)
{
    int8_t i = 0;
    for(i = 0; str[i]!= '\0';i++)
    {
        ST7735_DrawCharS(row*8 +6*i*size, col * 6*size , str[i], front, back, size);
    }

}


void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE,status);

    if (status & EUSCI_A_IE_RXIE)
    {

        U2RXData = MAP_UART_receiveData(EUSCI_A2_BASE);

        if(bufferposition >= 2500)
            bufferposition = 0;
        receivingBuffer [bufferposition++] = U2RXData;
        while(!(UCA0IFG & UCTXIFG));
        UCA0TXBUF = U2RXData;

    }
}

int checkforOK()
{
    //printf("Response: %s",receivingBuffer);
   char* charPointer = strstr(receivingBuffer,"OK");
    //lastIndex = bufferposition;

    if (charPointer != NULL)
        return 1;
    else
        return 0;

}

int checkforClosed()
{
    //printf("Response: %s",receivingBuffer);
    char* charPointer = strstr(receivingBuffer,"CLOSED");
    //lastIndex = bufferposition;

    if (charPointer != NULL)
        return 1;
    else
        return 0;

}

int checkforError()
{
    //printf("Response: %s",receivingBuffer);
    char* charPointer = strstr(receivingBuffer,"ERROR");
    //lastIndex = bufferposition;

    if (charPointer != NULL)
        return 1;
    else
        return 0;

}

int alreadyConnected()
{
    char* charPointer = strstr(receivingBuffer,"ALREADY CONNECTED");
    //lastIndex = bufferposition;

    if (charPointer != NULL)
        return 1;
    else
        return 0;
}



char* getTime()
{
    char *timePointer = strstr(receivingBuffer,"+IPD,51:");
    if (timePointer != NULL)
        return timePointer;
    else
        return 0;
}

void updateDateAndTime(){
    char* pointintTotime;
    pointintTotime = getTime();
    if(pointintTotime){
        memcpy(timeBuffer,pointintTotime,32);
        getDateAndTime();
    }

}

void getDateAndTime()
{
    newTime.year = (int)(timeBuffer[15]-48)*10+(int)(timeBuffer[16]-48);
    newTime.month = (int)(timeBuffer[18]-48)*10+(int)(timeBuffer[19]-48);
    newTime.dayOfmonth = (int)(timeBuffer[21]-48)*10+(int)(timeBuffer[22]-48);
    newTime.hours = (int)(timeBuffer[24]-48)*10+(int)(timeBuffer[25]-48);
    newTime.minutes = (int)(timeBuffer[27]-48)*10+(int)(timeBuffer[28]-48);

    if (newTime.hours == 0)
    {
        newTime.hours = 24;
        newTime.dayOfmonth -=1;
    }
    newTime.hours -= 4;

    MAP_RTC_C_initCalendar(&newTime, RTC_C_FORMAT_BINARY);//passing the entered input in RTC in binary format

    /* Specify an interrupt to assert every minute */
    MAP_RTC_C_setCalendarEvent(RTC_C_CALENDAREVENT_MINUTECHANGE);
    MAP_RTC_C_startClock();
  //  getSensorValues(&temperature,&pressure,&humidity,&current_solar, &voltage, &current_ltc, &voltage2, &voltage3, &current_battery);


    timeset = 1;
    DelayWait10ms(5);
    ST7735_FillScreen(ST7735_BLACK);
    CurrentState = ConnectSpreadSheet;
 //
//    CurrentState = END;

}


int checkForIPD()
{
    char* ipdPointer = strstr(receivingBuffer,"+IPD");
    if (ipdPointer != NULL)
            return 1;
        else
            return 0;

}



void send_to_ESP8266(char *sendStr)
{   int i;
   // printf(" Sending: %s",sendStr);
    for(i=0;i<strlen(sendStr);i++)
        MAP_UART_transmitData(EUSCI_A2_BASE,sendStr[i]);
}

void resetBuffer()
{
    memset(receivingBuffer,0,2500);
    bufferposition = 0;
}


void displayData(){
    if(screen==screen1)
           {
        if(gotdata==1)
        {
            gotdata=2;
            ST7735_FillScreen(ST7735_BLACK);

        }


       writecommand(ST7735_DIS0N);
      MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);

        if(timeset==1)
        {
                     sprintf(displayBuff,"%.2d/%.2d/%.2d", currentTime.year, currentTime.month, currentTime.dayOfmonth);
                       display(0,0,displayBuff,ST7735_GREEN,ST7735_BLACK,1);

                        if (timeBlinker)
                             sprintf(displayBuff,"%.2d %.2d", currentTime.hours, currentTime.minutes);
                        else
                             sprintf(displayBuff,"%.2d:%.2d", currentTime.hours, currentTime.minutes);


                        display(10,0,displayBuff,ST7735_GREEN,ST7735_BLACK,1);
        }
                        display(0,5,"Temp:", ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%s Cel", temperature);
                        display(8,5,displayBuff, ST7735_GREEN,ST7735_BLACK,1);

                        display(0,7,"Pressure:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%s inHg", pressure);
                        display(8,7,displayBuff, ST7735_GREEN,ST7735_BLACK,1);
                        display(0,9,"Humidity:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%s %%rh", humidity);
                        display(8,9,displayBuff, ST7735_GREEN,ST7735_BLACK,1);


                        display(0,13, "Solar Panel:", ST7735_GREEN,ST7735_BLACK,1);
                        display(0,15,"I:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%s", current_solar);
                        display(2,15,displayBuff, ST7735_GREEN,ST7735_BLACK,1);
                        display(7,15, "mA", ST7735_GREEN,ST7735_BLACK,1);
                        display(10,15,"V:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%sV", voltage);
                        display(12,15,displayBuff, ST7735_GREEN,ST7735_BLACK,1);

                        display(0,17, "LTC:", ST7735_GREEN,ST7735_BLACK,1);
                        display(0,19,"I:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%s", current_ltc);
                        display(2,19,displayBuff, ST7735_GREEN,ST7735_BLACK,1);
                        display(7,19,"mA", ST7735_GREEN,ST7735_BLACK,1);

                        display(10,19,"V:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%sV", voltage2);
                        display(12,19,displayBuff, ST7735_GREEN,ST7735_BLACK,1);


                        display(0,21, "Battery:", ST7735_GREEN,ST7735_BLACK,1);
                        display(0,23,"I:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%s", current_battery);
                        display(2,23, displayBuff,ST7735_GREEN,ST7735_BLACK,1);
                        display(7,23, "mA",ST7735_GREEN,ST7735_BLACK,1);

                        display(10,23,"V:",ST7735_CYAN,ST7735_BLACK,1);
                        sprintf(displayBuff,"%sV", voltage3);
                        display(12,23,displayBuff, ST7735_GREEN,ST7735_BLACK,1);
                        MAP_PCM_gotoLPM0();


                 MAP_PCM_gotoLPM0();
                 // MAP_PCM_gotoLPM3();


         }

           else if(screen== screen2)
           {
             MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
             writecommand(ST7735_DISPOFF);
             MAP_PCM_gotoLPM3();

        }
}


void CS_Init(void)
{
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    PCM_setCoreVoltageLevel(PCM_VCORE1);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);
}


void ESP8266_HardReset(void)
{
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);

    DelayWait10ms(10);

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
}




void main(void)
{

        WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

        boardInit();
        clk_init();
        port6_init();
        CS_Init();

        port1_init();
        rtc_init();
        MAP_Interrupt_enableMaster();
        adcInit();
        bme280Init();
        ST7735_InitR(INITR_REDTAB);//enabling LCD
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);

        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);

      //  MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
        // writecommand(ST7735_DISPOFF);
      //  display(0,7,"Loading",ST7735_GREEN,ST7735_BLACK,2);
         //
      //  uartA0_init();
         uartA2_init();


         DelayWait10ms(5);


        CurrentState = CheckAdc;

            while(1)
            {
                switch(CurrentState){

                case CheckAdc:
                {
                    getSensorValues(&temperature,&pressure,&humidity,&current_solar, &voltage, &current_ltc, &voltage2, &voltage3, &current_battery);
                    DelayWait10ms(5);
                    batteryVoltage= (int)(voltage3[0]-48);
                    if(batteryVoltage>=2)
                 {
                        CurrentState=  ESPReset;

             }
                   else
                    {
                         CurrentState= END;
                       less_battery=1;
                    }

                    resetBuffer();
                    break;

                }
                case ESPReset:
                {
                    {
                        ESP8266_HardReset();
                        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);
                        DelayWait10ms(5);
                        ST7735_FillScreen(ST7735_BLACK);
                        CurrentState =ConnectionSetUp;
                        connection_entered=1;

                    }

                    resetBuffer();
                    break;

                }


                    case ConnectionSetUp:
                         {


                             /*Hard Reset ESP8266*/
                          less_battery=2;
                         DelayWait10ms(5);

                          display(0,7,"Loading",ST7735_GREEN,ST7735_BLACK,2);

                      //    DelayWait10ms(5);
                          send_to_ESP8266("AT\r\n");
                          DelayWait10ms(5);
                          if(checkforOK())
                          {
                              send_to_ESP8266("AT+RST\r\n");
                              CurrentState = SetESPMode;
                              DelayWait10ms(5);

                          }
                          resetBuffer();
                          break;
                         }

                    case SetESPMode:
                    {
                        send_to_ESP8266("AT+CWMODE=1\r\n");

                        DelayWait10ms(5);
                        if(checkforOK())
                          {
                               CurrentState = ConnectToRouter;

                              DelayWait10ms(5);
                          }
                        resetBuffer();

                          break;
                    }
                    case ConnectToRouter:
                    {
                        sprintf(at_commands,"AT+CWJAP=\"%s\",\"%s\"\r\n",SSID,password);
                        send_to_ESP8266(at_commands);
                        DelayWait10ms(2000);
                        if(checkforOK())
                        {
                            CurrentState = TCPConnect;
                            DelayWait10ms(200);

                        }
                        resetBuffer();
                        break;

                    }

                    case TCPConnect:
                    {
                        while(!timeset){
                            send_to_ESP8266("AT+CIPSTART=\"TCP\",\"time.nist.gov\",13\r\n");
                            DelayWait10ms(200);

                            if (alreadyConnected())
                            {
                                send_to_ESP8266("AT+CIPCLOSE\r\n");
                            }
                            if(checkforClosed())
                            {
                                updateDateAndTime();
                            }
                            else
                            {
                                DelayWait10ms(300);
                            }

                    }
                        resetBuffer();
                        break;
                    }


                    case ConnectSpreadSheet:
                    {
                        if( batteryVoltage< 1)
                        {
                            CurrentState= END;
                            resetBuffer();
                            break;
                        }

                        clk_init();

                        uartA2_init();
                        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);
                        send_to_ESP8266("AT+CIPSTART=\"TCP\",\"api.pushingbox.com\",80\r\n");
                        DelayWait10ms(200);
                        if(checkforOK())
                        {
                            CurrentState = UpdateData;
                            DelayWait10ms(5);

                        }
                        resetBuffer();
                        break;

                     }

                    case UpdateData:
                    {

//                        sprintf(get_command,"GET /pushingbox?devid=%s&humidityData=%s&celData=%s&hicData=%s HTTP/1.1\r\nHost: api.pushingbox.com\r\nUser-Agent: ESP8266/1.0\r\nConnection:close \r\n\r\n",deviceId,humidity,temperature,pressure);

                        sprintf(get_command,"GET /pushingbox?devid=%s&humidityData=%s&celData=%s&fehrData=%s&currentSolar=%s&currentLTC=%s&currentBattery=%s&voltageSolar=%s&voltageLTC=%s&voltageBattery=%s HTTP/1.1\r\nHost: api.pushingbox.com\r\nUser-Agent: ESP8266/1.0\r\nConnection:close \r\n\r\n",deviceId,humidity,temperature,pressure,current_solar, current_ltc, current_battery, voltage, voltage2, voltage3);



                        sprintf(at_commands,"AT+CIPSEND=%d\r\n",strlen(get_command));
                        send_to_ESP8266(at_commands);
                        DelayWait10ms(10);
                        send_to_ESP8266(get_command);
                        DelayWait10ms(200);
                        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
                        connection_entered=0;
                        spreadsheet=0;
                        gotdata=1;
                            CurrentState = END;
                        resetBuffer();
                        break;

                    }


                    case END:
                    {

                        getSensorValues(&temperature,&pressure,&humidity,&current_solar, &voltage, &current_ltc, &voltage2, &voltage3, &current_battery);


                        batteryVoltage= (int)(voltage3[0]-48);
                        if(less_battery==1)
                        {

                            if(batteryVoltage>=1)
                            {

                                CurrentState= ESPReset;
                                writecommand(ST7735_DIS0N);
                                MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
                                DelayWait10ms(5);
                                resetBuffer()
                                ;
                                break;

                            }
                        }

                        displayData();
                        break;
                }


                    default:
                        break;

                }
            }
}




void RTC_C_IRQHandler(void)
{
    uint32_t status;

    status = MAP_RTC_C_getEnabledInterruptStatus();
    MAP_RTC_C_clearInterruptFlag(status);

    if (status & RTC_C_CLOCK_READ_READY_INTERRUPT)//taking interrupt in each second
    {


      //MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        currentTime = MAP_RTC_C_getCalendarTime();//storing the updated value
        timeBlinker = !timeBlinker;
        if(connection_entered!=1)
             {
            if(spreadsheet!=1)
            {
            timerDelay++;

        if(timerDelay==2)

        {
           timerDelay=0;
            CurrentState=END;
         }
            }

        screenOff++;
             if(screenOff==20)
             {
               screenOff=0;
            if(screen== screen1)
            {
               screen=screen2;
              }
               }



            }

       }

    if (status & RTC_C_TIME_EVENT_INTERRUPT)
    {//taking interrupt in each minute
   //     wakeup++
       twoSecondToggle++;



        if(twoSecondToggle == 60)
        {
            twoSecondToggle = 0;
            if(less_battery!=1)
              {
                spreadsheet= 1;
                CurrentState = ConnectSpreadSheet;

              }
        }

    }

    if (status & RTC_C_CLOCK_ALARM_INTERRUPT)//interrupts when time reaches alarm time
    {

    }

}




/* GPIO ISR */
void PORT1_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);


    /* Toggling the output on the LED */
    if(status & GPIO_PIN1)

    {
        if(screen==screen1)
        {
            screen= screen2;
        }

    }

    if(status & GPIO_PIN4)
    {
        if(screen==screen2)
        {
            screen=screen1;
        }
    }
}

