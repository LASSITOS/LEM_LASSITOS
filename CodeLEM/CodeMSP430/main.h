/*
 * main.h
 *
 *      Author: Thimira Asurapmudalige
 */

#ifndef MAIN_H_
#define MAIN_H_

/************************************ SD ************************************************/
#define SD_SIGNATURE    0x2A2A2A2A

/************************************* ADC pins and register directory**************************/
#define ADC_CTRL_PORT   P5OUT
#define ADC_CTRL_DIR    P5DIR
#define ADC_START_PIN   BIT4
#define ADC_RST_PIN     BIT5

#define ADC_CS_PORT     P1OUT
#define ADC_CS_DIR      P1DIR
#define ADC1_CS_PIN     BIT2
#define ADC2_CS_PIN     BIT0
#define ADC3_CS_PIN     BIT4

#define ADC_DRDY_PORT   P1OUT
#define ADC_DRDY_DIR    P1DIR
#define ADC_DRDY_IN     P1IN
#define ADC_DRDY_IE     P1IE
#define ADC_DRDY_IFG    P1IFG
//#defone ADC_DRDY_IES    P1IES
#define ADC1_DRDY_PIN   BIT3
#define ADC2_DRDY_PIN   BIT1
#define ADC3_DRDY_PIN   BIT5

#define ADC_SPI_PORT    P9OUT
#define ADC_SPI_SEL     P9SEL
#define ADC_SCLK_PIN    BIT4
#define ADC_SIMO_PIN    BIT5
#define ADC_SOMI_PIN    BIT6

/**********************************  **********************************/
#define SD_CS_PORT      P8OUT
#define SD_CS_DIR       P8DIR
#define SD_CS_PIN       BIT7

#define UART_SEL        P8SEL
#define UART_TX_PIN     BIT2
#define UART_RX_PIN     BIT3

#define LED1_PORT       P1OUT
#define LED1_DIR        P1DIR
#define LED2_PORT       P3OUT
#define LED2_DIR        P3DIR
#define LED3_PORT       P3OUT
#define LED3_DIR        P3DIR
#define LED1_PIN        BIT7
#define LED2_PIN        BIT0
#define LED3_PIN        BIT1


/***************************************************************************/
#define TCMD_START      0x81
#define TCMD_STOP       0x82
#define TCMD_RESET      0x84
#define TCMD_STATUS     0xFF

#define TSTAT_CMD_OK        0x80
#define TSTAT_CALIB_FAILED  0x20
#define TSTAT_SD_FULL       0x10
#define TSTAT_CALIBRATING   0x08
#define TSTAT_STARTING      0x04
#define TSTST_SD_ERR        0x02
#define TSTAT_RUNNING       0x01

#define TCMD_UART_START     'S'
#define TCMD_UART_RESET     'R'
#define TCMD_UART_STOP      'T'
#define TCMD_UART_STATUS    'Q'
#define TCMD_UART_GAIN      'G'
#define TCMD_UART_VERSION   'V'

#define TCMD_GAIN_1         '0'
#define TCMD_GAIN_2         '1'
#define TCMD_GAIN_4         '2'
#define TCMD_GAIN_8         '3'
#define TCMD_GAIN_16         '4'
#define TCMD_GAIN_32         '5'



enum state {CALIBRATING, DONE_CALIBRATING, RUNNING, NOT_RUNNING, SD_ERROR, SD_FULL, STARTING, READY, FAILED_CALIBRATING, FW_VERSION};

const char MSG_CALIBRATING[] = "CALIBRATING\r\n";   // 13
const char MSG_DONE_CALIBRATING[] = "CALIBRATION_DONE\r\n"; // 18
const char MSG_FAILED_CALIBRATING[] = "CALIBRATION_FAILED\r\n"; // 20
const char MSG_RUNNING[] = "RUNNING\r\n";   // 9
const char MSG_NOT_RUNNING[] = "NOT RUNNING\r\n";   // 13
const char MSG_SD_ERROR[] = "SD ERROR\r\n";   // 10
const char MSG_SD_FULL[] = "SD FULL\r\n";   //    9
const char MSG_STARTING[] = "STARTING\r\n"; //10
const char MSG_READY[] = "READY\r\n";   // 7





/***********************************************************************/
#define T_TRIG1_OUT    P3OUT
#define T_TRIG1_DIR     P3DIR
#define T_TRIG1_PIN     BIT7

#define T_TRIG1_OUT    P8OUT
#define T_TRIG1_DIR     P8DIR
#define T_TRIG1_PIN     BIT4

#define T_TRIG2_OUT    P3OUT
#define T_TRIG2_DIR     P3DIR
#define T_TRIG2_PIN     BIT5

#define T_TRIG_LIMIT    6000



#endif /* MAIN_H_ */
