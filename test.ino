/**
 * tdchung
 * tdchung.9@gmail.com
 * xx - Jun - 2019
 */


#include <UART_Interface.h>
#include <stdio.h>
#include <iwdg.h>


#ifndef GNSS_PWR_PIN
#define GNSS_PWR_PIN                   28 // PB12
#endif

#ifndef bool
#define bool                           int
#define true                           1 
#define false                          0
#endif

#define MAX_NMEA_FRAME_STRING          1023
#define MAX_NMEA_FRAME_SIZE            MAX_NMEA_FRAME_STRING + 1
#define MAX_POISITION_LENGHT           127

#define TIMER4_CYCLE_DEFAULT_MS        5000000  // 10s
#define TIMER3_CYCLE_DEFAULT_MS        1000000  // 1s


static HardwareTimer timer3(3);
static HardwareTimer timer4(4);

char* GgaInfo[] = {"0 xxGGA", "1 time", "2 lat", "3 NS", "4 long",
                   "5 EW", "6 quality", "7 numSV", "8 HDOP", "9 alt",
                   "10 uAlt", "11 sep", "12 uSep", "13 diffAge", 
                   "14 diffStation", "15 cs", "16 <CR><LF>", NULL};

char* GllInfo[] = {"0 xxGLL", "1 lat", "2 NS", "3 long", "4 EW", 
                   "5 time", "6 status", "7 posMode", "8 cs",
                   "9 <CR><LF>", NULL};

char* GnsInfo[] = {NULL};

char** GnssFrame[] = {GgaInfo, GllInfo, GnsInfo, NULL};

//--------------------------------------------------------------------------------------------------
// strAppendCharater, NOTE modify buff
// TODO: bug if lenght out of range.
void strAppendCharater(char *buff, char insert, int pos)
{
    char temp[MAX_NMEA_FRAME_SIZE] = {0};
    
    // printf ("BUFF before: %s\n", buff);

    strncpy(temp, buff, pos); // copy at most first pos characters
    // printf ("temp 1: %s\n", temp);

    *(temp+pos) = insert;
    // printf ("temp 2: %s\n", temp);
    
    strcpy((temp+pos+1), buff+pos); 
    // printf ("temp 3: %s\n", temp);

    strcpy(buff, temp);   // copy it back to subject
    
    // printf ("BUFF after : %s\n", buff);
}

//--------------------------------------------------------------------------------------------------
// update nmea buffer
// TODO: bug if lenght out of range.
void updateNmeaBuff(char *buff)
{
    int len = strlen(buff);
    // printf("Lenght: %d\n",len);
    for(int i=0; i<len-1; i++)
    {
        if (',' == *(buff+i) && (',' == *(buff+i+1)))
        {
            //  printf("i : %d\n",i);
            strAppendCharater(buff, '#', i+1);
        }
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * get gnss Nmea Field
 */
//--------------------------------------------------------------------------------------------------
static bool getNmeaField(char *nmea, int indexField, char *valueOut)
{
    char *token = NULL;
    char *token_ptr = NULL;
#ifdef __cplusplus
    char* nmea_buff = (char*)malloc(MAX_NMEA_FRAME_SIZE);
#else
    char* nmea_buff = malloc(MAX_NMEA_FRAME_SIZE);
#endif
    memset(nmea_buff, 0, MAX_NMEA_FRAME_SIZE);
    
    strncpy(nmea_buff, nmea, MAX_NMEA_FRAME_SIZE);

    // // now start get line
    // if (0 == indexField)
    // {
    //     if ( NULL != (token = strtok_r(nmea_buff, ",", &token_ptr)))
    //     {
    //         // got data
    //         strncpy(valueOut, token, MAX_POISITION_LENGHT);
    //         goto getNmeaField_Passed;
    //     }
    //     else
    //         goto getNmeaField_Failed;
    // }
    
    token_ptr = nmea_buff;
    // else
    // {
    for (int i=0; i<indexField+1; i++)
    {
        if ( NULL != (token = strtok_r(token_ptr, ",", &token_ptr)))
        // (token = strtok_r(token_ptr, ",", &token_ptr));
        {
            if (indexField == i)
            {
                // got data
                strncpy(valueOut, token, MAX_POISITION_LENGHT);
                goto getNmeaField_Passed;
            }
        }
        else
            goto getNmeaField_Failed;
    }
    // }

getNmeaField_Passed:
    free(nmea_buff);
    return true;
getNmeaField_Failed:
    free(nmea_buff);
    return false;
}

//--------------------------------------------------------------------------------------------------
/**
 * get gnss nmea specific frame from nmea gnss buffer
 */
//--------------------------------------------------------------------------------------------------
static bool getNmeaFrame (char *buff, char *nmea, char *output, size_t lenght)
{
    char *substr = NULL;
    char *token = NULL;
    char *token_ptr = NULL;
#ifdef __cplusplus
    char *nmea_buff = (char*)malloc(MAX_NMEA_FRAME_SIZE);
#else
    char *nmea_buff = malloc(MAX_NMEA_FRAME_SIZE);
#endif

    memset(nmea_buff, 0, MAX_NMEA_FRAME_SIZE);

    strncpy(nmea_buff, buff, MAX_NMEA_FRAME_STRING);
    substr = strstr (nmea_buff, nmea);
    if (NULL == substr)
    {
        SerialUSB.println ("==getNmeaFrame== GNSS frame not available");
        goto getNmeaFrame_False;
    }
    else
    // nmea is available
    // SerialUSB.println ("substr = : %s\n", substr);

    // now start get line
    token = strtok_r(substr, "\r", &token_ptr);
    // SerialUSB.println("token = : %s\n", token);
    // SerialUSB.println("token_ptr = : %s\n", token_ptr);
    if (NULL == token)
    {
        SerialUSB.println("==getNmeaFrame== get token failed\n");
        goto getNmeaFrame_False;
    }

    // got data
    strncpy(output, token, lenght);

    free(nmea_buff);
    return true;
getNmeaFrame_False:
    free(nmea_buff);
    return false;
}

//--------------------------------------------------------------------------------------------------
/**
 * test, analyse the nmea frame
 */
//--------------------------------------------------------------------------------------------------
static bool analyseGnssFrame(char *nmea)
{
    char data[MAX_POISITION_LENGHT] = {0};
    char str_out[MAX_NMEA_FRAME_SIZE] = {0};
    int index = 0;

    // working on temp buff.
#ifdef __cplusplus
    char *nmea_buff = (char*)malloc(MAX_NMEA_FRAME_SIZE);
#else
    char *nmea_buff = malloc(MAX_NMEA_FRAME_SIZE);
#endif
    memset(nmea_buff, 0, MAX_NMEA_FRAME_SIZE);
    strncpy(nmea_buff, nmea, MAX_NMEA_FRAME_STRING);

    // update the nmea buffer first
    // snprintf(str_out, MAX_NMEA_FRAME_SIZE, "NMEA before: %s", nmea_buff);
    // SerialUSB.println(str_out);

    updateNmeaBuff(nmea_buff);

    // snprintf(str_out, MAX_NMEA_FRAME_SIZE, "NMEA after: %s", nmea_buff);
    // SerialUSB.println(str_out);

    if (NULL != strstr(nmea_buff, "GNS"))
    {
        // max Field 15
        SerialUSB.println("NMEA: GNSS fix data GNS \n");
        index = 2;
        // for(int i=0; i<15; i++)
        // {
        //     if (getNmeaField(nmea_buff, i, data))
        //     {
        //         // SerialUSB.println(data[0]);
        //         if ('#' == data[0])
        //         {
        //             // SerialUSB.println("Data Field  is null");
        //             // strncpy
        //             strncpy(data, "null", 5);
        //         }
        //         snprintf(str_out, MAX_NMEA_FRAME_SIZE, "GNS: Field %d: %s", i, data);
        //         // SerialUSB.print("Field %d: ", i);
        //         // SerialUSB.print("Field ");
        //         SerialUSB.println(str_out);
        //     }
        // }
        // int i = 0;
        // while (NULL != GnsInfo[i])
        // {
        //     if (getNmeaField(nmea_buff, i, data))
        //     {
        //         if ('#' == data[0])
        //             strncpy(data, "null", 5);

        //         snprintf(str_out, MAX_NMEA_FRAME_SIZE, "GNS: Field %s: %s", GnsInfo[i], data);
        //         SerialUSB.println(str_out);
        //     }
        //     i++;
        // }
    }

    else if (NULL != strstr(nmea_buff, "GLL"))
    {
        // max Field 9
        SerialUSB.println("NMEA: GNSS Latitude and longitude, with time of position fix and status GLL \n");
        index =1;
        // for(int i=0; i<9; i++)
        // {
        //     if (getNmeaField(nmea_buff, i, data))
        //     {
        //         if ('#' == data[0])
        //         {
        //             // SerialUSB.println("Data Field  is null");
        //             // strncpy
        //             strncpy(data, "null", 5);
        //         }
        //         snprintf(str_out, MAX_NMEA_FRAME_SIZE, "GLL: Field %d: %s", i, data);
        //         SerialUSB.println(str_out);
        //     }
        // }
    }

    else if (NULL != strstr(nmea_buff, "GGA"))
    {
        // max Field 16
        SerialUSB.println("NMEA: GNSSLatitude Global positioning system fix data GGA \n");
        index = 0;
        // for(int i=0; i<16; i++)
        // {
        //     if (getNmeaField(nmea_buff, i, data))
        //     {
        //         if ('#' == data[0])
        //         {
        //             // SerialUSB.println("Data Field  is null");
        //             // strncpy
        //             strncpy(data, "null", 5);
        //         }
        //         snprintf(str_out, MAX_NMEA_FRAME_SIZE, "GGA: Field %d: %s", i, data);
        //         SerialUSB.println(str_out);
        //     }
        // }
    }

    else
    {
        SerialUSB.println("NMEA: not supported \n");
        return false;
    }

    int i = 0;
    while (NULL != GnssFrame[index][i])
    {
        if (getNmeaField(nmea_buff, i, data))
        {
            if ('#' == data[0])
                strncpy(data, "null", 5);

            snprintf(str_out, MAX_NMEA_FRAME_SIZE, "GNS: Field %s: %s", GnssFrame[index][i], data);
            SerialUSB.println(str_out);
        }
        i++;
    }

    return true;
}

//--------------------------------------------------------------------------------------------------
/**
 * setup fuction, arduino
 */
//--------------------------------------------------------------------------------------------------
void setup()
{
    // gnss configuration
    // 1. setup gnss baudrate
    // 2. enable gnss power
    SerialGNSS.begin(SerialGNSS_BAUDRATE);
    pinMode(GNSS_PWR_PIN, OUTPUT);
    digitalWrite(GNSS_PWR_PIN, HIGH);

    // timer 3 and 4 with channel 1
    timer3.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
    timer3.pause();
    timer3.setPeriod(TIMER3_CYCLE_DEFAULT_MS);
    timer3.attachInterrupt(TIMER_CH1, timer3_handler);
    timer3.refresh();
    timer3.resume();

    timer4.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
    timer4.pause();
    timer4.setPeriod(TIMER4_CYCLE_DEFAULT_MS);
    timer4.attachInterrupt(TIMER_CH1, timer4_handler);
    timer4.refresh();
    timer4.resume();

    iwdg_init(IWDG_PRE_256, 1250); // init an 8 second wd timer
}

//--------------------------------------------------------------------------------------------------
/**
 * Main loop arduino
 */
//--------------------------------------------------------------------------------------------------
void loop()
{
    // do something here
}

//--------------------------------------------------------------------------------------------------
/**
 * timer 3 interrupt handler
 */
//--------------------------------------------------------------------------------------------------
void timer3_handler(void)
{
    char gnss_frame[1024] = {0};
    char *gnss_ptr = gnss_frame;

    static int i = 0; i++;
    SerialUSB.print("INFO: TIMER 3 HANDLER: ");
    SerialUSB.println(i);

    while(SerialGNSS.available())
    {
        // SerialDebug.write(SerialGNSS.read());
        char c = SerialGNSS.read();
        *gnss_ptr = c;
        gnss_ptr++;
    }

    // print nmea debug
    // SerialUSB.println("--------------------------------");
    // SerialUSB.println(gnss_frame);
    // SerialUSB.println("--------------------------------");

    // get frame
    char gga_frame[MAX_NMEA_FRAME_SIZE] = {0};
    if (getNmeaFrame(gnss_frame, "GNGGA", gga_frame, MAX_NMEA_FRAME_SIZE))
    {
        // get frame OK
        SerialUSB.print("GOT GNGGA: "); SerialUSB.println(gga_frame);
        analyseGnssFrame(gga_frame);
    }
    else
    {
        SerialUSB.print("FAILED TO GET GNGGA");
    }
    
    
    char gll_frame[MAX_NMEA_FRAME_SIZE] = {0};
    if (getNmeaFrame(gnss_frame, "GNGLL", gll_frame, MAX_NMEA_FRAME_SIZE))
    {
        // get frame OK
        SerialUSB.print("GOT GNGLL: "); SerialUSB.println(gll_frame);
        analyseGnssFrame(gll_frame);
    }
    else
    {
        SerialUSB.println("FAILED TO GET GNGLL");
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * timer 4 interrupt handler
 */
//--------------------------------------------------------------------------------------------------
void timer4_handler(void)
{
    static int i = 0;
    i++;
    SerialUSB.print("===========::============   timer4 handler: ");
    SerialUSB.println(i);
    // watchdog
    iwdg_feed();
}
