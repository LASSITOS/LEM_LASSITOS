/*
 * main.c
 *
 *      Author: Thimira Asurapmudalige
 */

#define VERSION "3.1.0"

#include "driverlib.h"
#include "mmc.h"
#include <string.h>
#include "diskio.h"
#include "msp430.h"
#include <stdlib.h>
#include "main.h"
#include <stdio.h>

#define TRUE    1

#define FALSE   0

#define BUFFERSZ    512     // Data buffer size
#define NUM_BUFFERS 60      // Number of data buffers


/***************************** ADC Configurations ****************************/
#define ADC_SPEED   0x0E    // 19200SPS
volatile uint8_t adc_gain = 0x00;


/*****************************  *************************/
#define UCS_MCLK_DESIRED_KHZ    20000
#define UCS_MCLK_FLL_RATIO      610


/***************************** DMA Configuration ****************************/
#define DMA_ADC_DA  (DMA_BASE + DMA_CHANNEL_0 + OFS_DMA0DA)     // DMA ADC receive channel Destination address


/***************************** File system configuration *******************/
#define     FILE_NAME_LENGTH    12

typedef struct fileInfo_t{
    char        fileName[FILE_NAME_LENGTH];  // filename
    uint32_t    startAddress;  // Start address of the file
    uint32_t    ADC_Frame_count;    // Number of ADC data frames
    uint32_t    SD_Frame_count;     // Number of SD data frames
    //uint16_t     usedFile;
}fileInfo_t;

typedef struct FileSys_t{
    uint32_t    signature;
    uint8_t    filePtr;
    uint8_t    isDownloaded;
    fileInfo_t  fileInformation[16];
}FileSys_t;

FileSys_t curFileSys = {0};

uint32_t sd_blockSize, sd_sectorCount;

fileInfo_t file;



/************************************* Global Variable Declarations ***************************************/

volatile uint8_t curADCBuf = 0, curSDBuf = 0;   // current data receiving buffer and SD card writing buffer
volatile uint16_t dataPtr = 0;                  // current data receiving pointer
volatile uint32_t ADCFrames = 0;                // Number of ADC frames assembled
volatile uint32_t SDFrames = 0;                 // Number of SD frames sent to SD
volatile uint32_t sampleNum = 0;                // Sample counter
volatile uint8_t dataADC[BUFFERSZ * NUM_BUFFERS];   // data buffer for the ADC data
volatile uint8_t * dataADCPtrs[NUM_BUFFERS];    // array containing pointers for the data buffer frames
volatile char RXDataUART;                       // RX data receive
volatile char RXDataUART_T;                       // RX data receive from T
volatile char RXDataSPIT;                       // RX data from SPI_T
volatile uint8_t TStatus = 0x00;                      //
volatile uint8_t isTCMDReceived;                   // SPI T command received
volatile uint8_t isTCMDStart = FALSE;                    // current command
volatile uint8_t isTCMDGain = FALSE;                    // is command gain
volatile char TFileName[FILE_NAME_LENGTH];      // file name
volatile uint8_t TcharCnt=0;
volatile uint16_t i=0, j=0, k=0;                // variables for indexing
volatile uint8_t Running = FALSE;                // Sampling is happening or not
volatile uint8_t CalibCounter = 0;

volatile uint8_t RunningMainLoop = FALSE;       // This determines whether the main loop is running or not
volatile uint8_t SDSyncNow = FALSE;             // Sync with SD card now

volatile DRESULT res = RES_OK;                  // disk operation result

const uint8_t DUMMY =  0x00;    // dummy byte containing all zeros

char uartBuf[200], number[15];

uint8_t RWBuffer[BUFFERSZ];             // SD card read buffer

volatile uint16_t T_trig_cnt;
volatile uint8_t SDSyncCounter = 0;

DSTATUS res2;

/****************************** test ****************************/
volatile int32_t tempADCVal = 0, res_cos32=0, res_sin32=0;
uint8_t sin_cos_ptr = 0;
int16_t ref_sin[32] = {0,   211,   235,    50,  -180,  -249,   -97,   141,   254,   141,   -97,  -249,  -180,    50,   235,   211,     0,  -211,  -235,   -50,   180,   249,    97,  -141,  -254,  -141,    97,   249,   180,   -50,  -235,  -211};
int16_t ref_cos[32] = {254, 141,   -97,  -249,  -180,    50,   235,   211,     0,  -211,  -235,   -50,   180,   249,    97,  -141,  -254,  -141,    97,   249,   180,   -50,  -235,  -211,     0,   211,   235,    50,  -180,  -249,   -97,   141};
volatile int64_t mres0=0 ,mres1=0 ,mres2=0 ,mres3=0 ,res_cos=0 ,res_sin=0, rx64 = 0, tx64 = 0, res64 = 0, sum64 = 0;
const int MAX_SAMPLES_LOCKIN = 1960;  // max number of samples to add
volatile int num_samples_lockin = 0;  // to track how many samples now



/****************************************************Function Declaration***********************************/
void initUART(void);
void initUARTforT(void);                            // initialize UART for top board
void WriteUARTMsg(const char * msg);    //
void WriteUARTMsgT(const char * msg);
void WriteUARTMsgT_Fixed(uint8_t msg_id);
void initSPIforADC(void);                           // SPI initialize for ADC
void initSPIforT(void);                             // SPI initialize for top board
void initDMA(void);                                 // initialize DMA
void readSDparam(void);
void WriteUARTHex(uint8_t hex);                     // Write to UART hex value
void WriteADCRegCalib(uint8_t Addr, uint8_t data);  // Write to ADC registers for calibration
void initADC(void);                                 // initialize ADC
int8_t CalibrateADC(void);                            // calibrate ADCs
void getFileInfoFromSD(void);
void sdTestWrite(void);
void WriteADCReg(uint8_t Addr, uint8_t data);       // Write to ADC registers
uint8_t ReadADCRegD1(uint8_t Addr);
//uint8_t ReadADCRegD2(uint8_t Addr);
uint8_t ReadADCRegD3(uint8_t Addr);




/**************************************************** Main Function ***************************************/

void main (void)
{
    //Port select XT1
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P7, GPIO_PIN2 + GPIO_PIN3 );


    WDT_A_hold(WDT_A_BASE);                         // Hold watchdog timer

    /*******************************Power level and clock configurations *************************************************************/
        PMM_setVCore(PMMCOREV_3);                       // Set VCore to level 3
        UCS_setExternalClockSource(32768, 7372800);           // Set XT1 frequency as 32768 and XT Frequency as 0
        UCSCTL6 &= ~(XCAP_3 + XT2OFF + XT1OFF);

        UCSCTL6 &= ~(XCAP_3 + XT1OFF);
        //while(UCSCTL7 & XT1LFOFFG){
        //    UCSCTL7 &= ~(XT1LFOFFG);
        //    SFRIFG1 &= ~OFIFG;
        //}
        UCSCTL1 = DCORSEL_6;
        UCSCTL2 = UCS_MCLK_FLL_RATIO;
        UCSCTL3 = 0;

        UCSCTL5 = 0;


        UCSCTL6 &= ~(XT2DRIVE_3 + XT2OFF);

        while(UCSCTL7 & XT2OFFG){
            UCSCTL7 &= ~XT2OFFG;
            SFRIFG1 &= ~OFIFG;
        }

        UCSCTL4 = SELS_5 + SELM_4 + SELA_4;

        while(UCSCTL7 & DCOFFG){
            UCSCTL7 &= ~DCOFFG;
            SFRIFG1 &= ~OFIFG;
        }

/********************************* Port mapping *************************************************************************/
        //const uint8_t port_mapping2[] = {PM_NONE, PM_UCA0CLK, PM_UCA0SIMO,PM_UCA0SOMI, PM_UCA0STE, PM_NONE, PM_NONE, PM_NONE};      // mapping
        const uint8_t port_mapping2[] = {PM_UCA0TXD, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_UCA0RXD};      // mapping

        PMAP_initPortsParam initPortsParam2 = {0};
        initPortsParam2.portMapping = port_mapping2;
        initPortsParam2.PxMAPy = (uint8_t *) &P2MAP01;
        initPortsParam2.numberOfPorts = 1;
        initPortsParam2.portMapReconfigure = PMAP_ENABLE_RECONFIGURATION;
        PMAP_initPorts(PMAP_CTRL_BASE, &initPortsParam2);

/****************************** GPIO Configuration ********************************************************************/
        PAOUT = 0;
        PBOUT = 0;
        PCOUT = 0;
        PDOUT = 0;
        PEOUT = 0;

        P1DIR |= /*BIT0 ADC2_CS*   |*/ BIT2 /*ADC1_CS*/      | BIT4 /*ADC3_CS*/      | BIT7 /*LED1*/; //P1 output pins
        P1OUT |= /*BIT0 ADC2_CS*   |*/ BIT2 /*ADC1_CS*/      | BIT4 /*ADC3_CS*/  ; // make ADC_CS pins high
        P1REN |= /*BIT1 ADC2_DRDY* |*/ BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/;   // resistor enable on input pins
        P1OUT |= /*BIT1 ADC2_DRDY* |*/ BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/;   // pullup resistor on input pins
        P1IES |= /*BIT1 ADC2_DRDY* |*/ BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/;   // interrupt edge select
        //P1IE  |= BIT1 /*ADC2_DRDY*/ | BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/;   // enable interrupts on data ready pins

        P1IFG = 0;  // clear port 1 interrupt flags

        //P2SEL = 0b00011110;           // Except LSB select all other pins for special function in port 2
        P2SEL = 0b10000001;           // special function in port 2
        //P2DIR |= BIT4;

        P3DIR |= BIT0 /*LED2*/      | BIT1 /*LED3*/         | BIT4 /*ADC_MCLK*/; // output pins port 2
        P3SEL |= BIT4 /*ADC_MCLK*/; // special function select port 2
        P3OUT |= BIT0 /*LED2*/ ;

        P5DIR |= BIT4 /*ADC_START*/ | BIT5 /*ADC_RST*/ ;
        P5OUT |= BIT5 /*ADC_RST*/;                                      //

        P6DIR |= BIT0|BIT1|BIT2|BIT3;

        P8DIR |= BIT1 /*SD_POWER*/  | BIT7 /*SD_CS*/;    // PORT 8 output pins
        P8SEL |= BIT2 /*UART_TX*/   | BIT3 /*UART_RX*/;         // Special function select for UART RX and TX pins
        P8OUT |= BIT1 /*SD_POWER*/;     // power on the SD card

        P9SEL = 0b01111110;         // Except LSB select all other pins for special function in port 9
        //P9REN |= BIT0 /*SD_DET*/ ;//| BIT2 /*SD_SIMO*/ | BIT3 /*SD_SOMI*/;          // resistor enable on port 9
        //P9OUT |= BIT0 /*SD_DET*/ ;//| BIT2 /*SD_SIMO*/ | BIT3 /*SD_SOMI*/;   // Pull up port 9 pin


        T_TRIG1_DIR |= T_TRIG1_PIN;     // Trigger signals for GPS module
        T_TRIG2_DIR |= T_TRIG2_PIN;
        T_TRIG1_OUT |= T_TRIG1_PIN;
        T_TRIG2_OUT |= T_TRIG2_PIN;


////////////////////////////////////////////////////////////////////////////////////////////////////
        initUART();         // init UART
        initSPIforADC();    // initialize spi for ADC
       // initSPIforT();      // initialize SPI for top board
        initUARTforT();     // init UART for top board
        initDMA();          // initialize DMA
        __delay_cycles(20000);


        WriteUARTMsg("UART Enabled\n\r");
        sprintf(&uartBuf, "FW Version: %s", VERSION);
        WriteUARTMsg(uartBuf);

        memset(&uartBuf, 0, sizeof(uartBuf));
        ltoa(sizeof(FileSys_t), number,10);
        strcpy(uartBuf, "\r\nSize of FileSys_t : ");
        strcat(uartBuf, number);
        WriteUARTMsg(uartBuf);

        if(BIT0 /*SD_DET*/){
            WriteUARTMsg("\r\nInitializing SD card. \r\n");
            for(i=0;i<10;i++){
                WriteUARTMsg("AT \r\n");
                res2 = disk_initialize(0);
                if(res2 == 0){
                    break;
                }
                __delay_cycles(100);
            }
            if(res2 == 0){
                WriteUARTMsg("SD Card initialized. \r\n");
                P1OUT |= BIT7 /*LED1*/;
                res = disk_read(0, &RWBuffer[0], 0, 1);
                memcpy(&curFileSys, &RWBuffer, sizeof(FileSys_t));
                if(curFileSys.signature == SD_SIGNATURE){
                    if(curFileSys.isDownloaded == TRUE){
                        WriteUARTMsg("Already downloaded. Clearing the files.\r\n");
                        curFileSys.filePtr = 0;
                        //memcpy(&RWBuffer[0], &curFileSys, sizeof(FileSys_t));
                        //res = disk_write(0, &RWBuffer[0], 0, 1);
                    }

                     WriteUARTMsg("File system found\r\n");
                     memset(&uartBuf, 0, sizeof(uartBuf));
                     sprintf(&uartBuf, "Files Available : %d\n\r", curFileSys.filePtr);
                     WriteUARTMsg(uartBuf);
                }else{
                    WriteUARTMsg("File system not found, Creating structure.\r\n");
                    memset(&curFileSys, 0, sizeof(FileSys_t));
                    memset(&RWBuffer[0], 0, 512);
                    curFileSys.signature = SD_SIGNATURE;
                    curFileSys.filePtr = 0;
                    memcpy(&RWBuffer[0], &curFileSys, sizeof(FileSys_t));
                    res = disk_write(0, &RWBuffer[0], 0, 1);
                }

            }else{
                P3OUT |= BIT1 /*LED3*/;
                TStatus |= TSTST_SD_ERR;
                WriteUARTMsg("Failed to initialize SD card.\r\n");
                //while(1);
            }
        }else{
            WriteUARTMsg("SD card not available. \r\n");
            TStatus |= TSTST_SD_ERR;
            //whle(1);
        }
        __delay_cycles(1000);

        readSDparam();                  // read SD card number of sectors and number of blocks

        memset((void*)&dataADC[0], 0, BUFFERSZ * NUM_BUFFERS);
        //uint8_t * dataADCPtrs[6] = {&dataADC1[0], &dataADC2[0], &dataADC3[0], &dataADC4[0], &dataADC5[0], &dataADC6[0]};
        for(i=0;i<NUM_BUFFERS;i++){
            dataADCPtrs[i] = &dataADC[i*BUFFERSZ];
        }

        //memset(&file1, 0, sizeof(fileInfo_t));
        //strcpy(file1.fileName, "file1");
        //file1.startAddress = 2;

        __enable_interrupt();
//       while(1){
//        res_sin32 = -5000000;
//        res_cos32 = 100000;
//        mres0 += ((int64_t)res_sin32 * (int64_t)res_cos32)>>16;
//
//       }
//        while(1){
//            int64_t t1 = 0x23232323, t2 = 0x14141414;
//            memcpy(dataADCPtrs[curADCBuf]+dataPtr+4, &t1, 4 );
//            memcpy(dataADCPtrs[curADCBuf]+dataPtr+8, &t2, 4 );
//            P6OUT |= BIT0;
////            memcpy(&rx64,dataADCPtrs[curADCBuf]+dataPtr+4, 4 );
////            memcpy(&tx64,dataADCPtrs[curADCBuf]+dataPtr+8, 4 );
////            res64 = *(dataADCPtrs[curADCBuf]+dataPtr+4) * *(dataADCPtrs[curADCBuf]+dataPtr+8);
////            rx64 = *(int32_t*)(dataADCPtrs[curADCBuf]+dataPtr+4);
////            tx64 = *(int32_t*)(dataADCPtrs[curADCBuf]+dataPtr+8);
////            res64 = rx64 * tx64;
//            res64 = (int64_t)(*(int32_t*)(dataADCPtrs[curADCBuf]+dataPtr+4)) * (int64_t)(*(int32_t*)(dataADCPtrs[curADCBuf]+dataPtr+8));
//            sum64 += res64>>16;
//            P6OUT &= ~BIT0;
//        }



        if(curFileSys.filePtr>=16){
            WriteUARTMsg("File system full.");
            WriteUARTMsgT("File system full.");
            TStatus |= TSTAT_SD_FULL;
            while(1);
        }

        WriteUARTMsg("\n\rInit Done\n\r");
        //WriteUARTMsgT("\n\rInit Done\n\r");
       // WriteUARTMsgT_Fixed(READY);

        while(1)
        {
            if(curADCBuf != curSDBuf)                                // if a buffer is full
            {
                P6OUT |= BIT0;
                if(disk_write_multiple((const uint8_t *) dataADCPtrs[curSDBuf])==RES_OK) SDFrames++;  // If the writing success, increment the SD Frames
                curSDBuf++;     // Move to the next SD buffer
                if(curSDBuf>=NUM_BUFFERS) {
                    curSDBuf=0;   // reset the cycle
                    SDSyncCounter++;
                    if(SDSyncCounter>= 100){
                        SDSyncCounter = 0;
                        SDSyncNow = TRUE;
                    }
                }
                if((RunningMainLoop == FALSE) && (curADCBuf == curSDBuf)){                   // if the Stop sample has been received
                    __delay_cycles(1000);
                    disk_Write_multiple_stop();         // stop multiple write to SD card
                    TStatus &= ~TSTAT_RUNNING;
                    //UCA0TXBUF = TStatus;

                    memset(&uartBuf, 0, sizeof(uartBuf));
                    ltoa(ADCFrames, number,10);
                    strcpy(uartBuf, "\r\nData ADC frames : ");
                    strcat(uartBuf, number);
                    WriteUARTMsg(uartBuf);
                    WriteUARTMsgT(uartBuf);

                    memset(&uartBuf, 0, sizeof(uartBuf));
                    ltoa(SDFrames, number,10);
                    strcpy(uartBuf, "\r\nSD data frames : ");
                    strcat(uartBuf, number);
                    WriteUARTMsg(uartBuf);
                    WriteUARTMsgT(uartBuf);

                    //file1.ADC_Frame_count = ADCFrames;
                    //file1.SD_Frame_count = SDFrames;
                    curFileSys.fileInformation[curFileSys.filePtr-1].ADC_Frame_count = ADCFrames;
                    curFileSys.fileInformation[curFileSys.filePtr-1].SD_Frame_count = SDFrames;
                    //curFileSys.filePtr++;
                    memset(&RWBuffer[0],0,BUFFERSZ);
                    //memcpy(&RWBuffer[0], &file1, sizeof(fileInfo_t));
                    memcpy(&RWBuffer[0], &curFileSys, sizeof(FileSys_t));
                    if(disk_write(0, &RWBuffer[0], 0, 1)== RES_OK){
                        WriteUARTMsg("\n\rFile Created Successfully.\n\r");
                        WriteUARTMsgT("\n\rFile Created Successfully.\n\r");
                    }else{
                        WriteUARTMsg("\n\rFile create failed.\n\r");
                        WriteUARTMsgT("\n\rFile Created Successfully.\n\r");
                        TStatus |= TSTST_SD_ERR;
                    }
                    //UCA0TXBUF = TStatus;
                    T_TRIG1_OUT |= T_TRIG1_PIN;        // Reset GPS trigger signal 1
                }
                P6OUT &= ~BIT0;
            }else{
                if(SDSyncNow == TRUE){                 // if want to sync now
                    P6OUT |= BIT1;

                    SDSyncNow = FALSE;
                    disk_Write_multiple_stop();
                    curFileSys.fileInformation[curFileSys.filePtr-1].ADC_Frame_count = ADCFrames;
                    //file.ADC_Frame_count = ADCFrames;
                    curFileSys.fileInformation[curFileSys.filePtr-1].SD_Frame_count = SDFrames;
                    //file.SD_Frame_count = SDFrames;
                    //memcpy(&RWBuffer[0], &file1, sizeof(fileInfo_t));
                    memcpy(&RWBuffer[0], &curFileSys, sizeof(FileSys_t));
                    disk_write(0, &RWBuffer[0], 0, 1);
                    //disk_write_multiple_init(file1.startAddress + SDFrames);
                    disk_write_multiple_init(curFileSys.fileInformation[curFileSys.filePtr-1].startAddress + SDFrames);
                    P6OUT &= ~BIT1;
                }
            }

        }
}

/*********************************************************** Interrupt Service Routines *****************************************************************/
// ADC functional
#pragma vector=PORT1_VECTOR
__interrupt
void port1_ISR(void)
{
    if((ADC_DRDY_IFG & ADC1_DRDY_PIN) && /*(ADC_DRDY_IFG & ADC2_DRDY_PIN) && */ (ADC_DRDY_IFG & ADC3_DRDY_PIN)) //&& (P1IFG & BIT3 /*ADC1_DRDY*/) && (P1IFG & BIT5 /*ADC3_DRDY*/))
    {
        P6OUT |= BIT2;
        dataPtr += 16;
        sin_cos_ptr++;      // test
        if(dataPtr >= BUFFERSZ)     // if the buffer filled, switch the current ADC data buffer
        {
            T_trig_cnt++;
            T_TRIG2_OUT |= T_TRIG2_PIN;            // reset trigger signal
            if(T_trig_cnt >= T_TRIG_LIMIT){
                T_trig_cnt = 0;
                T_TRIG2_OUT &= ~T_TRIG2_PIN;       // trigger signal 2 for gps
            }
            //P2OUT |= BIT4;
            T_TRIG1_OUT |= T_TRIG1_PIN;        // Reset GPS trigger signal 1
            dataPtr = 0;        // reset data pointer
            sin_cos_ptr = 0;        // test
            res_cos = 0;            //test
            res_sin = 0;            //test
            ADCFrames++;        // increment ADC frame number
            curADCBuf++;        // move to next ADC buffer
            if(curADCBuf>=NUM_BUFFERS) curADCBuf = 0;       // move to first buffer of circuilar bufer

            if(Running == FALSE)        // if the stop command received stop sampling
            {
                P5OUT &= ~BIT4 /*ADC_START*/;       // Stop ADC
                T_TRIG1_OUT &= ~ T_TRIG1_PIN;      // trigger GPS timer
                P1IE &=  ~(/*BIT1 *ADC2_DRDY*/  BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/);     // disable ADC DRDY interrupt
                P1IFG &= ~(/*BIT1 *ADC2_DRDY*/  BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/);     // clear ADC DRDY interrupt flags
                RunningMainLoop = FALSE;
                LED3_PORT &= ~LED3_PIN;
                //P2OUT &= ~BIT4;
                return;
            }
            //P2OUT &= ~BIT4;
        }

        sampleNum++;        // Increment the sample number
        memcpy((void*)(dataADCPtrs[curADCBuf]+dataPtr),(void*) &sampleNum, 4);
        DMA_setDstAddress(DMA_CHANNEL_0, (uint32_t)dataADCPtrs[curADCBuf]+dataPtr+4, DMA_DIRECTION_INCREMENT);

        UCB2IFG &= ~UCRXIFG;            // clear RX interrupt flag
        DMA0CTL |= DMAEN;               // Enable DMA transfer on RX (8 bytes)

        P1OUT &= ~BIT2 /*ADC1_CS*/;     // ADC1 Chip select
        DMA1CTL |= DMAEN;               // Enable DMA transfer on TX (4 bytes)
        __delay_cycles(80);             // wait till transfer done
        P1OUT |= BIT2 /*ADC1_CS*/;     // ADC1 Chip deselect


        P1OUT &= ~BIT4 /*ADC3_CS*/;      // ADC3 Chip select
        DMA1CTL |= DMAEN;               // Enable DMA transfer on TX (4 bytes)
        __delay_cycles(80);             // wait till transfer done
        P1OUT |= BIT4 /*ADC3_CS*/;      // ADC3 Chip deselect




        ADC_DRDY_IFG &= ~(ADC1_DRDY_PIN | ADC3_DRDY_PIN);       // Clear Data ready flags
        P6OUT &= ~BIT2;


//        memcpy(&rx64,dataADCPtrs[curADCBuf]+dataPtr+4, 4 );
//        memcpy(&tx64,dataADCPtrs[curADCBuf]+dataPtr+4, 4 );
//        res64 = rx64 * tx64;

       /* P6OUT |= BIT1;
        DMA_setSrcAddress(DMA_CHANNEL_4, (uint32_t)dataADCPtrs[curADCBuf]+dataPtr+7, DMA_DIRECTION_DECREMENT);          //// test
        DMA4CTL |= DMAEN | DMAREQ;
        __delay_cycles(4);
        //MPY32


        //MPYS32L = tempADCVal & 0xFFFF;
        //MPYS32H = (tempADCVal>>16) & 0xFFFF;
        res_sin32 = -5000000;
        res_cos32 = 100000;
        mres0 = res_sin32 * res_cos32;


        OP2 = ref_cos[sin_cos_ptr];
        __delay_cycles(8);
        mres0 = RES0;
        mres1 = RES1;
        mres2 = RES2;
        mres3 = RES3;
        res_cos += mres3<<48 | mres2<<32 | mres1<<16 | mres0;

        OP2 = ref_sin[sin_cos_ptr];
        __delay_cycles(8);
        mres0 = RES0;
        mres1 = RES1;
        mres2 = RES2;
        mres3 = RES3;
        res_sin += mres3<<48 | mres2<<32 | mres1<<16 | mres0;
        P6OUT &= ~BIT1;*/

    }
}


//UART ISR
#pragma vector=USCI_A1_VECTOR
__interrupt

void EUSCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV, USCI_UCTXIFG))
    {
    case USCI_NONE: break;
    case USCI_UCRXIFG:
        RXDataUART = USCI_A_UART_receiveData(USCI_A1_BASE);
        if(RXDataUART == 'S')       // start sampling
        {
            if(curFileSys.filePtr>=16){
                WriteUARTMsg("File system full.");
                return;
            }
            if(curFileSys.filePtr==0){
                //curFileSys.filePtr++;
                curFileSys.fileInformation[curFileSys.filePtr].startAddress = 5;
                //file.startAddress = 5;
            }else{
                //file.startAddress = curFileSys.fileInformation[curFileSys.filePtr-1].startAddress + curFileSys.fileInformation[curFileSys.filePtr-1].SD_Frame_count + 2;
                curFileSys.fileInformation[curFileSys.filePtr].startAddress = curFileSys.fileInformation[curFileSys.filePtr-1].startAddress + curFileSys.fileInformation[curFileSys.filePtr-1].SD_Frame_count + 2;
            }
            char tempBuf[20];
            sprintf(&tempBuf, "UART SMPL %d", curFileSys.filePtr);
            strncpy(&curFileSys.fileInformation[curFileSys.filePtr].fileName[0], &tempBuf, 12);
            curFileSys.filePtr++;
            //memcpy(&curFileSys.fileInformation[curFileSys.filePtr], &file, sizeof(fileInfo_t));
            memcpy(&RWBuffer[0], &curFileSys, sizeof(FileSys_t));
            res = disk_write(0, &RWBuffer[0], 0, 1);

            //file1.startAddress = 2;
            res = disk_write_multiple_init(curFileSys.fileInformation[curFileSys.filePtr-1].startAddress);
            if(res == RES_OK)
            {
                WriteUARTMsg("\r\nStart Sampling\r\n");
                RunningMainLoop = TRUE;
                //P1IFG &= ~(BIT1 /*ADC2_DRDY*/ | BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/);    //  Clear ADC DRDY interrupt flags
                ADC_DRDY_IFG &= ~(ADC1_DRDY_PIN/* | ADC2_DRDY_PIN */| ADC3_DRDY_PIN);
                //P1IE |= (BIT1 /*ADC2_DRDY*/ | BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/);      //  Enable ADC DRDY interrupt flags
                ADC_DRDY_IE |= (ADC1_DRDY_PIN /*| ADC2_DRDY_PIN */| ADC3_DRDY_PIN);
                Running = TRUE;
                LED3_PORT |= LED3_PIN;
                T_TRIG1_OUT &= ~T_TRIG1_PIN;                    // Trigger for GPS timing
                P5OUT |= BIT4 /*ADC_START*/;                    // Start ADC
            }else{
                WriteUARTMsg("\r\nCannot open file\r\n");
            }
        }
        else if(RXDataUART == 'T')                  // Stop sampling
        {
            Running = FALSE;
        }
        else if(RXDataUART == 'E')                  // Read ADC Registers
        {
            for(i=0;i<0x1A;i++)
            {
                uint8_t adcReg1 = ReadADCRegD1(i);
                //uint8_t adcReg2 = ReadADCRegD2(i);
                uint8_t adcReg3 = ReadADCRegD3(i);
                WriteUARTMsg("\r\n");
                WriteUARTHex(i);WriteUARTMsg("\t");
                WriteUARTHex(adcReg1);WriteUARTMsg("\t");
                //WriteUARTHex(adcReg2);WriteUARTMsg("\t");
                WriteUARTHex(adcReg3);WriteUARTMsg("\t");
            }
        }
        else if(RXDataUART == 'D')              // INIT ADC
        {
            initADC();
        }
        else if(RXDataUART == 'W')              // test write to sd card
        {
            //testReadWriteSD();
        }
        else if(RXDataUART == 'F')              // Fetch file info
        {
            getFileInfoFromSD();
        }
        else if(RXDataUART == 'N')              // Create new file in SD card
        {
            ADCFrames = 0;
            dataPtr = 0;
            sampleNum = 0;
            SDFrames = 0;
        }
        else if(RXDataUART == 'C')              // Calibrate ADCs
        {
            if(CalibrateADC() == FALSE){
                WriteUARTMsg("\r\nCALIBRATION FAILED.\r\n");
                return;
            }
        }
        else if(RXDataUART == 'G')              // Calibrate ADCs
        {
            adc_gain = 3;
            WriteUARTMsg("\n\rGain Set.\n\r");
        }
        else if(RXDataUART == 'R'){
            WDT_A_start(WDT_A_BASE);
            while(1);
        }

        RXDataUART = 0;
        break;

    case USCI_UCTXIFG: break;
    //case USCI_UCSTTIFG: break;
   // case USCI_UART_UCTXCPTIFG: break;
    }
}

/********************************************************** UART T ISR ****************************************************************************/
//UART ISR
#pragma vector=USCI_A0_VECTOR
__interrupt

void EUSCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, USCI_UCTXIFG))
    {
    case USCI_NONE: break;
    case USCI_UCRXIFG:
        RXDataUART_T = USCI_A_UART_receiveData(USCI_A0_BASE);
      //  WriteUARTMsg(RXDataUART_T);
        if(RXDataUART_T == TCMD_UART_STATUS){

            isTCMDStart = FALSE;
            isTCMDGain = FALSE;
            if(Running == TRUE){
                WriteUARTMsgT_Fixed(RUNNING);
            }else{


                if(TStatus & TSTST_SD_ERR) WriteUARTMsgT_Fixed(SD_ERROR);
                if(TStatus & TSTAT_SD_FULL) WriteUARTMsgT_Fixed(SD_FULL);
                if(TStatus & TSTAT_CALIB_FAILED) WriteUARTMsgT_Fixed(FAILED_CALIBRATING);
                if(TStatus & TSTAT_STARTING) WriteUARTMsgT_Fixed(STARTING);
                if(TStatus & TSTAT_CALIBRATING) WriteUARTMsgT_Fixed(CALIBRATING);
                WriteUARTMsgT_Fixed(READY);

            }
        }
        else if(RXDataUART_T==TCMD_UART_GAIN){
            isTCMDGain = true;
        }
        else if(RXDataUART_T == TCMD_UART_STOP){
            Running = FALSE;
        }
        else if(RXDataUART_T == TCMD_UART_VERSION){
            WriteUARTMsgT_Fixed(FW_VERSION);
        }
        else if(RXDataUART_T == TCMD_UART_START){
            /*if(curFileSys.filePtr>=16){
                return;
            }*/
            if(!((TStatus & TSTST_SD_ERR) || (TStatus & TSTAT_SD_FULL))){

                isTCMDStart = TRUE;
                memset((void *)TFileName,0,sizeof(TFileName));  // clear file name buffer
                TcharCnt = 0;                           // reset character count
                TStatus &= ~(TSTAT_STARTING | TSTST_SD_ERR | TSTAT_RUNNING);
            }
        }else if(RXDataUART_T == TCMD_UART_RESET){
            WDT_A_start(WDT_A_BASE);
            while(1);
        }
        else if(isTCMDGain == true){
            char gain = RXDataUART_T;
            if (gain == TCMD_GAIN_1)    adc_gain = 0;
            if (gain == TCMD_GAIN_2)    adc_gain = 1;
            if (gain == TCMD_GAIN_4)    adc_gain = 2;
            if (gain == TCMD_GAIN_8)    adc_gain = 3;
            if (gain == TCMD_GAIN_16)   adc_gain = 4;
            if (gain == TCMD_GAIN_32)   adc_gain = 5;
            WriteUARTMsg("\n\rGain Set.\n\r");
            isTCMDGain = false;
        }
        else if(isTCMDStart == TRUE){
            TFileName[TcharCnt] = RXDataUART_T;
            TcharCnt++;
            if(TcharCnt>=FILE_NAME_LENGTH){         // complete file name received
                isTCMDStart = FALSE;
                TStatus |= TSTAT_STARTING;          // update status
                if(curFileSys.filePtr>=16){
                    WriteUARTMsg("File system full.");
                    WriteUARTMsgT_Fixed(SD_FULL);
                    TStatus |= TSTAT_SD_FULL;
                    TStatus &= ~TSTAT_STARTING;
                    return;
                }
                if(curFileSys.filePtr==0){
                    curFileSys.fileInformation[curFileSys.filePtr].startAddress = 5;
                }else{
                    curFileSys.fileInformation[curFileSys.filePtr].startAddress = curFileSys.fileInformation[curFileSys.filePtr-1].startAddress + curFileSys.fileInformation[curFileSys.filePtr-1].SD_Frame_count + 2;
                }
                memcpy((void *)curFileSys.fileInformation[curFileSys.filePtr].fileName, (void *)&TFileName, FILE_NAME_LENGTH);
                curFileSys.filePtr++;

                memcpy(&RWBuffer[0], &curFileSys, sizeof(FileSys_t));
                res = disk_write(0, &RWBuffer[0], 0, 1);

                ADCFrames = 0;
                SDFrames = 0;
                dataPtr = 0;
                sampleNum = 0;

                initADC();

                WriteUARTMsgT_Fixed(CALIBRATING);
                __delay_cycles(100000);
                TStatus |= TSTAT_CALIBRATING;
                if(CalibrateADC() == FALSE){
                    TStatus &= ~TSTAT_STARTING;
                    TStatus &= ~TSTAT_CALIBRATING;
                    TStatus |= TSTAT_CALIB_FAILED;
                    WriteUARTMsgT_Fixed(FAILED_CALIBRATING);
                    WriteUARTMsg("\r\nCALIBRATION FAILED.\r\n");
                    return;
                }else{
                    WriteUARTMsgT_Fixed(DONE_CALIBRATING);
                }
                TStatus &= ~TSTAT_CALIBRATING;
                __delay_cycles(500000);
                res = disk_write_multiple_init(curFileSys.fileInformation[curFileSys.filePtr-1].startAddress);
                if(res == RES_OK)
                {
                    RunningMainLoop = TRUE;
                    ADC_DRDY_IFG &= ~(ADC1_DRDY_PIN |/* ADC2_DRDY_PIN |*/ ADC3_DRDY_PIN);                   //  Clear ADC DRDY interrupt flags
                    ADC_DRDY_IE |= (ADC1_DRDY_PIN /*| ADC2_DRDY_PIN */| ADC3_DRDY_PIN);                 //  Enable ADC DRDY interrupt flags
                    Running = TRUE;
                    TStatus &= ~(TSTAT_STARTING);                   // clear status starting bit
                    TStatus |= TSTAT_RUNNING;                       // set status running bit
                    //WriteUARTMsg("\r\nStart Sampling\r\n");
                    WriteUARTMsgT_Fixed(RUNNING);
                    //__delay_cycles(100000);
                    LED3_PORT |= LED3_PIN;
                    T_TRIG1_OUT &= ~T_TRIG1_PIN;                    // Trigger for GPS timing
                    ADC_CTRL_PORT |= ADC_START_PIN;                 // start ADC sampling
                } else {
                    TStatus |= TSTST_SD_ERR;                        // status SD error
                    //WriteUARTMsg("\r\nCannot open file\r\n");
                    WriteUARTMsgT_Fixed(SD_ERROR);
                }
            }
        }
        RXDataUART_T = 0;
        break;

    case USCI_UCTXIFG: break;
    //case USCI_UCSTTIFG: break;
   // case USCI_UART_UCTXCPTIFG: break;
    }
}

/********************************************************** SPI T ISR **************************************************/

//#pragma vector=USCI_A0_VECTOR
//__interrupt
void USCI_A0_ISR (void)
{
    switch (__even_in_range(UCA0IV,4))
    {
        case 2:
            RXDataSPIT = USCI_A_SPI_receiveData(USCI_A0_BASE);
            //WriteUARTHex(RXDataSPIT);
            if(RXDataSPIT == TCMD_STATUS){
                //WriteUARThex(RXDataSPIT);
                //WriteUARTMsg("\n\nSTATUS REQ \n\r");
                TStatus |= TSTAT_CMD_OK;
                UCA0TXBUF = TStatus;
                //UCA0TXBUF = 0b10010001;
                //WriteUARTHex(TStatus);
                //DMA3CTL |= DMAEN;
                TStatus &= ~TSTAT_CMD_OK;
                isTCMDStart = FALSE;
            }else if(RXDataSPIT == TCMD_STOP){
                Running = FALSE;
            }else if(RXDataSPIT == TCMD_START){
                if(curFileSys.filePtr>=16){
                    TStatus |= TSTAT_SD_FULL;
                    return;
                }
                //WriteUARTMsg("SI");
                isTCMDStart = TRUE;
                memset((void *)TFileName,0,sizeof(TFileName));  // clear file name buffer
                TcharCnt = 0;                           // reset character count
                TStatus &= ~(TSTAT_STARTING | TSTST_SD_ERR | TSTAT_RUNNING);
            }else if(RXDataSPIT == TCMD_RESET){
                WDT_A_start(WDT_A_BASE);
                while(1);
            }else if(isTCMDStart == TRUE){              // servising start command
                TFileName[TcharCnt] = RXDataSPIT;
                TcharCnt++;
                if(TcharCnt>=FILE_NAME_LENGTH){         // complete file name received
                    //WriteUARTMsg("SI");
                    isTCMDStart = FALSE;
                    TStatus |= TSTAT_STARTING;          // update status
                    //UCA0TXBUF = TStatus;
                    //memset((void *)&file, 0, sizeof(fileInfo_t));
                    if(curFileSys.filePtr>=16){
                        WriteUARTMsg("File system full.");
                        return;
                    }
                    if(curFileSys.filePtr==0){
                        //file.startAddress = 5;
                        curFileSys.fileInformation[curFileSys.filePtr].startAddress = 5;
                    }else{
                        curFileSys.fileInformation[curFileSys.filePtr].startAddress = curFileSys.fileInformation[curFileSys.filePtr-1].startAddress + curFileSys.fileInformation[curFileSys.filePtr-1].SD_Frame_count + 2;
                    }
                    //memcpy((void *)file.fileName,(void *)&TFileName,FILE_NAME_LENGTH);
                    memcpy((void *)curFileSys.fileInformation[curFileSys.filePtr].fileName, (void *)&TFileName, FILE_NAME_LENGTH);
                    curFileSys.filePtr++;
                    //memcpy(&curFileSys.fileInformation[curFileSys.filePtr], &file, sizeof(fileInfo_t));
                    memcpy(&RWBuffer[0], &curFileSys, sizeof(FileSys_t));
                    res = disk_write(0, &RWBuffer[0], 0, 1);


                    ADCFrames = 0;
                    SDFrames = 0;
                    dataPtr = 0;
                    sampleNum = 0;

                    initADC();

                    TStatus |= TSTAT_CALIBRATING;
                    //UCA0TXBUF = TStatus;
                    CalibrateADC();
                    TStatus &= ~TSTAT_CALIBRATING;
                    //UCA0TXBUF = TStatus;
                    res = disk_write_multiple_init(curFileSys.fileInformation[curFileSys.filePtr-1].startAddress);
                    if(res == RES_OK)
                    {
                        RunningMainLoop = TRUE;
                        //P1IFG &= ~(BIT1 /*ADC2_DRDY*/ | BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/);      //  Clear ADC DRDY interrupt flags
                        ADC_DRDY_IFG &= ~(ADC1_DRDY_PIN |/* ADC2_DRDY_PIN |*/ ADC3_DRDY_PIN);                   //  Clear ADC DRDY interrupt flags
                        //P1IE |= (BIT1 /*ADC2_DRDY*/ | BIT3 /*ADC1_DRDY*/    | BIT5 /*ADC3_DRDY*/);        //  Enable ADC DRDY interrupt flags
                        ADC_DRDY_IE |= (ADC1_DRDY_PIN /*| ADC2_DRDY_PIN */| ADC3_DRDY_PIN);                 //  Enable ADC DRDY interrupt flags
                        Running = TRUE;
                        //P5OUT |= BIT4 /*ADC_START*/;                  // Start ADC
                        TStatus &= ~(TSTAT_STARTING);                   // clear status starting bit
                        TStatus |= TSTAT_RUNNING;                       // set status running bit
                        //UCA0TXBUF = TStatus;
                        WriteUARTMsg("\r\nStart Sampling\r\n");
                        T_TRIG1_OUT &= ~T_TRIG1_PIN;                    // Trigger for GPS timing
                        ADC_CTRL_PORT |= ADC_START_PIN;                 // start ADC sampling
                    }else{
                        TStatus |= TSTST_SD_ERR;                        // status SD error
                        //UCA0TXBUF = TStatus;
                        WriteUARTMsg("\r\nCannot open file\r\n");
                    }

                }
            }
            RXDataSPIT = 0;
            break;

        default: break;

    }
}


/************************************ UART Functions ********************************************************************/
void initUART(void)
{
    /******************************** UART Interface for debugging ******************************************************/
       USCI_A_UART_initParam uartParam = {0};
       uartParam.selectClockSource = USCI_A_UART_CLOCKSOURCE_ACLK;

       uartParam.clockPrescalar = 10;
       uartParam.firstModReg = 0;
       uartParam.secondModReg = 14;

       uartParam.parity = USCI_A_UART_NO_PARITY;
       uartParam.msborLsbFirst = USCI_A_UART_LSB_FIRST;
       uartParam.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
       uartParam.uartMode = USCI_A_UART_MODE;
       uartParam.overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

       if(STATUS_FAIL == USCI_A_UART_init(USCI_A1_BASE, &uartParam)) return;

       USCI_A_UART_enable(USCI_A1_BASE);
       USCI_A_UART_clearInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
       USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
}

void WriteUARTMsg(const char * msg)                // send UART message for debugging
{
    int j;
    for(j=0; j<strlen(msg); j++){

        while(!(UCA1IFG & UCTXIFG));
        //UCA1TXBUF = msg[j];
        USCI_A_UART_transmitData(USCI_A1_BASE, msg[j]);
    }
}

void WriteUARTMsgT(const char * msg)                // send UART message for debugging
{
    int j;
    for(j=0; j<strlen(msg); j++){

        while(!(UCA0IFG & UCTXIFG));
        USCI_A_UART_transmitData(USCI_A0_BASE, msg[j]);
    }
}

void WriteUARTMsgT_Fixed(uint8_t msg_id)                // send UART message for Top Board
{

    switch (msg_id){
        case CALIBRATING:
            DMA3SZ = 13;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_CALIBRATING, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case DONE_CALIBRATING:
            DMA3SZ = 18;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_DONE_CALIBRATING, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case RUNNING:
            DMA3SZ = 9;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_RUNNING, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case NOT_RUNNING:
            DMA3SZ = 13;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_NOT_RUNNING, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case SD_ERROR:
            DMA3SZ = 10;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_SD_ERROR, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case SD_FULL:
            DMA3SZ = 9;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_SD_FULL, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case STARTING:
            DMA3SZ = 10;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_STARTING, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case READY:
            DMA3SZ = 7;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_READY, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case FAILED_CALIBRATING:
            DMA3SZ = 20;
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&MSG_FAILED_CALIBRATING, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;

        case FW_VERSION:
            memset(&uartBuf, 0, sizeof(uartBuf));
            sprintf(&uartBuf, "VERSION: %s",VERSION);
            DMA3SZ = strlen(uartBuf);
            DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&uartBuf, DMA_DIRECTION_INCREMENT);
            DMA3CTL |= DMAEN;
            break;
    }

}


void WriteUARTHex(uint8_t hex)                     // Write to UART hex value
{
    if((hex>>4)<10)
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = (hex>>4)+'0';
    }
    else
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = (hex>>4)+'7';
    }

    if((hex & 0x0F)<10)
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = (hex & 0x0F)+'0';
    }
    else
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = (hex & 0x0F)+'7';
    }
}

void initUARTforT(void)
{
    /******************************** UART Interface for Top board ******************************************************/
       USCI_A_UART_initParam uartParam = {0};
       uartParam.selectClockSource = USCI_A_UART_CLOCKSOURCE_ACLK;

       uartParam.clockPrescalar = 65;
       uartParam.firstModReg = 0;
       uartParam.secondModReg = 2;

       uartParam.parity = USCI_A_UART_NO_PARITY;
       uartParam.msborLsbFirst = USCI_A_UART_LSB_FIRST;
       uartParam.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
       uartParam.uartMode = USCI_A_UART_MODE;
       uartParam.overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

       if(STATUS_FAIL == USCI_A_UART_init(USCI_A0_BASE, &uartParam)) return;

       USCI_A_UART_enable(USCI_A0_BASE);
       USCI_A_UART_clearInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
       USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
}

/****************************************** SPI for ADC *******************************************************************/

void initSPIforADC(void)
{
    USCI_B_SPI_initMasterParam paramSPI = {0};
    paramSPI.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
    paramSPI.clockSourceFrequency = 7372800;
    paramSPI.desiredSpiClock = 7372800;
    paramSPI.msbFirst = USCI_B_SPI_MSB_FIRST;
    paramSPI.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    paramSPI.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    USCI_B_SPI_initMaster(USCI_B2_BASE, &paramSPI);
    USCI_B_SPI_enable(USCI_B2_BASE);
}


/****************************************** SPI for T *******************************************************************/
void initSPIforT(void)
{
    USCI_A_SPI_initSlave(USCI_A0_BASE,
                         USCI_A_SPI_MSB_FIRST,
                         //USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                         USCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
                         USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW
    );
    UCA0CTL0 |= UCMODE_2;

    //Enable SPI Module
    USCI_A_SPI_enable(USCI_A0_BASE);

    //Enable Receive interrupt
    USCI_A_SPI_clearInterrupt(USCI_A0_BASE,
                              USCI_A_SPI_RECEIVE_INTERRUPT
    );
    USCI_A_SPI_enableInterrupt(USCI_A0_BASE,
                               USCI_A_SPI_RECEIVE_INTERRUPT
    );

}


/******************************************* DMA Config *****************************************************/

void initDMA(void)
{
    DMA_initParam DParamADC = {0};
    DParamADC.channelSelect = DMA_CHANNEL_0;
    DParamADC.transferModeSelect = DMA_TRANSFER_SINGLE;
    DParamADC.transferSize = 8;
    DParamADC.triggerSourceSelect = DMA_TRIGGERSOURCE_14;   // UCB2RXIFG
    DParamADC.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
    DParamADC.triggerTypeSelect = DMA_TRIGGER_HIGH;
    DMA_init(&DParamADC);

    DParamADC.channelSelect = DMA_CHANNEL_1;
    DParamADC.transferSize = 4;
    DParamADC.triggerSourceSelect = DMA_TRIGGERSOURCE_15;   // UCB2TXIFG
    DMA_init(&DParamADC);

    DMA_setSrcAddress(DMA_CHANNEL_0, USCI_B_SPI_getReceiveBufferAddressForDMA(USCI_B2_BASE), DMA_DIRECTION_UNCHANGED);
    DMA_setDstAddress(DMA_CHANNEL_0, (uint32_t)(uintptr_t)&dataADC[0], DMA_DIRECTION_INCREMENT);

    DMA_setSrcAddress(DMA_CHANNEL_1, (uint32_t)(uintptr_t)&DUMMY, DMA_DIRECTION_UNCHANGED);
    DMA_setDstAddress(DMA_CHANNEL_1, USCI_B_SPI_getTransmitBufferAddressForDMA(USCI_B2_BASE), DMA_DIRECTION_UNCHANGED);

    DMA_initParam DParamSD = {0};
    DParamSD.channelSelect = DMA_CHANNEL_2;
    DParamSD.transferModeSelect = DMA_TRANSFER_SINGLE;
    DParamSD.transferSize = 512;
    DParamSD.triggerSourceSelect = DMA_TRIGGERSOURCE_13;    // UCA2TXIFG
    DParamSD.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
    DParamSD.triggerTypeSelect = DMA_TRIGGER_HIGH;
    DMA_init(&DParamSD);
    DMA_setDstAddress(DMA_CHANNEL_2, USCI_A_SPI_getTransmitBufferAddressForDMA(USCI_A2_BASE), DMA_DIRECTION_UNCHANGED);

    DMA_initParam DParamT = {0};
    DParamT.channelSelect = DMA_CHANNEL_3;
    DParamT.transferModeSelect = DMA_TRANSFER_SINGLE;
    DParamT.transferSize = 4;
    DParamT.triggerSourceSelect = DMA_TRIGGERSOURCE_17;
    DParamT.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
    DParamT.triggerTypeSelect = DMA_TRIGGER_HIGH;
    DMA_init(&DParamT);
    //DMA_setSrcAddress(DMA_CHANNEL_3, (uint32_t)(uintptr_t)&TSPIBuffer, DMA_DIRECTION_INCREMENT);
    DMA_setDstAddress(DMA_CHANNEL_3, USCI_A_UART_getTransmitBufferAddressForDMA(USCI_A0_BASE), DMA_DIRECTION_UNCHANGED);

    DMA_initParam DParamTemp = {0};
    DParamTemp.channelSelect = DMA_CHANNEL_4;
    DParamTemp.transferModeSelect = DMA_TRANSFER_BLOCK;
    DParamTemp.transferSize = 4;
    DParamTemp.triggerSourceSelect = DMA_TRIGGERSOURCE_0;
    DParamTemp.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
    DParamTemp.triggerTypeSelect = DMA_TRIGGER_HIGH;
    DMA_init(&DParamTemp);

    DMA_setDstAddress(DMA_CHANNEL_4, (uint32_t)(uintptr_t)&tempADCVal, DMA_DIRECTION_INCREMENT);

    DMACTL4 |= DMARMWDIS;
}




void readSDparam(void){
    disk_ioctl(0,GET_SECTOR_COUNT, &sd_sectorCount);
    disk_ioctl(0,GET_BLOCK_SIZE, &sd_blockSize);



    memset(&uartBuf, 0, sizeof(uartBuf));
    ltoa(sd_sectorCount, number,10);
    strcpy(uartBuf, "SD Card Number of Sectors : ");
    strcat(uartBuf, number);
    WriteUARTMsg(uartBuf);

    memset(&uartBuf, 0, sizeof(uartBuf));
    ltoa(sd_blockSize, number,10);
    strcpy(uartBuf, "\r\nSD Card block size : ");
    strcat(uartBuf, number);
    WriteUARTMsg(uartBuf);
}


/********************************************************************* SD Functions ************************************************/
void getFileInfoFromSD(void)
{
    res = disk_read(0, &RWBuffer[0], 0, 1);
    FileSys_t FSinfo = {0};
    memcpy(&FSinfo,&RWBuffer[0], sizeof(FileSys_t));


    //fileInfo_t fileInfo = {0};
    //memcpy(&fileInfo,&RWBuffer[0], sizeof(FileSys_t));
    memset(&uartBuf, 0, sizeof(uartBuf));
    strcpy(uartBuf, "\r\n\r\nNo.\t|FileName\t|StartAdd.\t|ADCFr\t|SDFr\t|\t\n\r");
    WriteUARTMsg(uartBuf);
    int j;
    for(j=0;j<16;j++){
        //char tbufSA[12]= {0};
        //char tbufFC[12]= {0};
        //ltoa((long)FSinfo.fileInformation[j].startAddress, tbufSA, 11);
        //ltoa((long)FSinfo.fileInformation[j].SD_Frame_count, tbufFC, 11);
        memset(&uartBuf, 0, sizeof(uartBuf));
        sprintf(&uartBuf, "%d\t|%s\t|\t%d\t|\t%d\t|\t%d\n\r",j, FSinfo.fileInformation[j].fileName, FSinfo.fileInformation[j].startAddress, FSinfo.fileInformation[j].ADC_Frame_count, FSinfo.fileInformation[j].SD_Frame_count);
       // sprintf(&uartBuf, "%d\t|%s\t|\t%s\t|\t%d\t|\t%s\n\r",j, FSinfo.fileInformation[j].fileName, FSinfo.fileInformation[j].startAddress, FSinfo.fileInformation[j].ADC_Frame_count, tbufFC);
        WriteUARTMsg(uartBuf);
    }
   /*
    strcpy(uartBuf, "\r\nFileName : ");
    strcat(uartBuf, fileInfo.fileName);
    strcat(uartBuf, "\r\nADCFrames : ");
    ltoa(fileInfo.ADC_Frame_count, number, 10);
    strcat(uartBuf, number);
    strcat(uartBuf, "\r\nSDFrames : ");
    ltoa(fileInfo.SD_Frame_count, number, 10);
    strcat(uartBuf, number);
    strcat(uartBuf, "\r\nStartAddress : ");
    ltoa(fileInfo.startAddress, number, 10);
    strcat(uartBuf, number);
    WriteUARTMsg(uartBuf);
    */

}


void sdTestWrite(void)
{

    //file1.ADC_Frame_count = 1359;
    //file1.SD_Frame_count = 2468;
    //file1.startAddress = 2;
    //memset(&RWBuffer[0],0,BUFFERSZ);
    //memcpy(&RWBuffer[0], &file1, sizeof(fileInfo_t));
    //if(disk_write(0, &RWBuffer[0], 0, 1)== RES_OK){
    //    WriteUARTMsg("\n\rFile Created Successfully.\n\r");
    //}else{
    //    WriteUARTMsg("\n\rFile create failed.\n\r");
    //}
}


/********************************************************************* ADC Functions ************************************************/

void initADC(void)              // initialize ADC
{
    WriteADCReg(0x01, 0x01);
    WriteADCReg(0x02, 0x00);
    WriteADCReg(0x03, 0x00);
    WriteADCReg(0x04, 0x80);
    WriteADCReg(0x05, (adc_gain<<4) | ADC_SPEED);
    WriteADCReg(0x06, 0x01);
    WriteADCReg(0x07, 0x00);
    WriteADCReg(0x08, 0x00);
    WriteADCReg(0x09, 0x00);
    WriteADCReg(0x0A, 0x00);
    WriteADCReg(0x0B, 0x00);
    WriteADCReg(0x0C, 0x40);
    WriteADCReg(0x0D, 0xBB);
    WriteADCReg(0x0E, 0x00);

    WriteADCReg(0x19, 0x00);
    WriteADCReg(0x1A, 0x40);

    WriteUARTMsg("Done INIT ADCs \r\n");
}

int8_t CalibrateADC(void)                            // calibrate ADCs
{
    WriteUARTMsg("Starting Calibration \r\n");
    ADC_DRDY_IE &= ~(ADC1_DRDY_PIN | /*ADC2_DRDY_PIN  | */ADC3_DRDY_PIN );    // disable interrupts on data ready pins
    ADC_DRDY_IFG &= ~(ADC1_DRDY_PIN | /*ADC2_DRDY_PIN | */ADC3_DRDY_PIN);    // clear interrupt on data ready pins
    //GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN6);
    //GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN6);

    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    ADC_CS_PORT &= ~ADC1_CS_PIN;                       // chip select ADC1
    WriteADCRegCalib(0x03, 0x00);                       // select continuous mode
    WriteADCRegCalib(0x05, (adc_gain<<4) |ADC_SPEED);                  // select ADC speed
    WriteADCRegCalib(0x01, 0x01);                       // select referance voltage
    WriteADCRegCalib(0x06, 0x01);                       // for system calibration select input to 01
    //GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);   // start conversion
    ADC_CTRL_PORT |= ADC_START_PIN;                     // start conversion
    __delay_cycles(10);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x16);      // start system offset calibration
    __delay_cycles(10);
    //while(GPIO_INPUT_PIN_HIGH == GPIO_getInputPinValue(GPIO_PORT_P1, ADC1_DRDY_PIN));   // wait till calibration done
    CalibCounter = 0;               // reset calibration counter
    while(ADC_DRDY_IN & ADC1_DRDY_PIN){                     // wait till calibration done
        __delay_cycles(10000);                              // wait 500 us
        CalibCounter++;
        if(CalibCounter>100){
            return FALSE;
        }
    }
    //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);    // stop conversion
    ADC_CTRL_PORT &= ~ADC_START_PIN;                      // stop conversion
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // deselect ADC1
    ADC_CS_PORT |= ADC1_CS_PIN;                          // deselect ADC1
    WriteUARTMsg("\n\rADC1 Calibrated.\n\r");
    __delay_cycles(1000);

/*    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);    // select ADC2
    ADC_CS_PORT &= ~ADC2_CS_PIN;                        // select ADC2
    WriteADCRegCalib(0x03, 0x00);                       // select continuous mode
    WriteADCRegCalib(0x05, ADC_SPEED);                  // select ADC speed
    WriteADCRegCalib(0x01, 0x01);                       // select referance voltage
    WriteADCRegCalib(0x06, 0x01);                       // for system calibration select input to 01
    //GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);   // start conversion
    ADC_CTRL_PORT |= ADC_START_PIN;                     // start conversion
    __delay_cycles(10);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x16);      // start system offset calibration
    __delay_cycles(10);
    //while(GPIO_INPUT_PIN_HIGH == GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4)); // wait till calibration done
    while(ADC_DRDY_IN & ADC2_DRDY_PIN);                 // wait till calibration done
    //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);    // stop conversion
    ADC_CTRL_PORT &= ~ADC_START_PIN;                      // stop conversion
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);   // deselect ADC2
    ADC_CS_PORT |= ADC2_CS_PIN;                          // deselect ADC2
     WriteUARTMsg("\n\rADC2 Calibrated.\n\r");

    __delay_cycles(1000);
    */

    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3);    // select ADC3
    ADC_CS_PORT &= ~ADC3_CS_PIN;                        // select ADC3
    WriteADCRegCalib(0x03, 0x00);                       // select continuous mode
    WriteADCRegCalib(0x05, (adc_gain<<4) |ADC_SPEED);                  // select ADC speed
    WriteADCRegCalib(0x01, 0x01);                       // select referance voltage
    WriteADCRegCalib(0x06, 0x01);                       // for system calibration select input to 01
    //GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);   // start conversion
    ADC_CTRL_PORT |= ADC_START_PIN;                     // start conversion
    __delay_cycles(10);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x16);      // start system offset calibration
    __delay_cycles(10);
    //while(GPIO_INPUT_PIN_HIGH == GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6));
    CalibCounter = 0;               // reset calibration counter
    while(ADC_DRDY_IN & ADC3_DRDY_PIN){                 // wait till calibration done
        __delay_cycles(10000);                          // wait 500 us
        CalibCounter++;
        if(CalibCounter>100){
            return FALSE;
        }
    }
    //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);    // stop conversion
    ADC_CTRL_PORT &= ~ADC_START_PIN;                    // stop conversion
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);   // deselect ADC3
    ADC_CS_PORT |= ADC3_CS_PIN;                         // deselect ADC3
    WriteUARTMsg("\n\rADC3 Calibrated.\n\r");

    __delay_cycles(1000);



    //GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN6);
    ADC_DRDY_IFG &= ~(ADC1_DRDY_PIN | /*ADC2_DRDY_PIN |*/ ADC3_DRDY_PIN);   // clear interrupt on ADC DRDY
    //GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN6);
    //ADC_DRDY_IE |= ADC1_DRDY_PIN | ADC2_DRDY_PIN | ADC3_DRDY_PIN;       // enable ADC DRDY interrupt
    WriteUARTMsg("Calibration completed. \r\n");
    return TRUE;
}

void WriteADCReg(uint8_t Addr, uint8_t data)       // Write to ADC registers
{
    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    ADC_CS_PORT &= ~ADC1_CS_PIN;            // Select ADC1
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x40);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x00);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, data);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
    __delay_cycles(20);
    ADC_CS_PORT |= ADC1_CS_PIN;             // deselect ADC1
    __delay_cycles(100);

 /*   //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    ADC_CS_PORT &= ~ADC2_CS_PIN;            // Select ADC2
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x40);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x00);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, data);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
    __delay_cycles(20);
    ADC_CS_PORT |= ADC2_CS_PIN;             // deselect ADC2
    __delay_cycles(100);*/

    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3);
    ADC_CS_PORT &= ~ADC3_CS_PIN;
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x40);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x00);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, data);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
    __delay_cycles(20);
    ADC_CS_PORT |= ADC3_CS_PIN;
    __delay_cycles(100);
}

void WriteADCRegCalib(uint8_t Addr, uint8_t data)  // Write to ADC registers for calibration
{
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x40);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x00);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, data);
}


uint8_t ReadADCRegD1(uint8_t Addr)                 // read ADC1 register
{
    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    ADC_CS_PORT &= ~ADC1_CS_PIN;                    // select ADC1
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x20);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x00);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_receiveData(USCI_B2_BASE);
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x00);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
    __delay_cycles(20);
    ADC_CS_PORT |= ADC1_CS_PIN;                     // deselect ADC1
    return USCI_B_SPI_receiveData(USCI_B2_BASE);
}
/*
uint8_t ReadADCRegD2(uint8_t Addr)                 // read ADC2 register
{
    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    ADC_CS_PORT &= ~ADC2_CS_PIN;                    // select ADC2
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x20);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x00);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_receiveData(USCI_B2_BASE);
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x00);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
    __delay_cycles(20);
    ADC_CS_PORT |= ADC2_CS_PIN;                     // deselect ADC2
    return USCI_B_SPI_receiveData(USCI_B2_BASE);
}*/

uint8_t ReadADCRegD3(uint8_t Addr)                 // read ADC2 register
{
    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3);
    ADC_CS_PORT &= ~ADC3_CS_PIN;                    // select ADC3
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, Addr + 0x20);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x00);
    while(!USCI_B_SPI_getInterruptStatus(USCI_B2_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    USCI_B_SPI_receiveData(USCI_B2_BASE);
    USCI_B_SPI_transmitData(USCI_B2_BASE, 0x00);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
    __delay_cycles(20);
    ADC_CS_PORT |= ADC3_CS_PIN;                     // deselect ADC3
    return USCI_B_SPI_receiveData(USCI_B2_BASE);
}

