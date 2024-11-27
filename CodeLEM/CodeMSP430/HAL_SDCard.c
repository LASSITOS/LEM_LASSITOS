/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * ======== HAL_SDCard.c ========
 */

/***************************************************************************//**
 * @file       HAL_SDCard.c
 * @addtogroup HAL_SDCard
 * Modified for LASSITOS Project by Thimira Asurapmudalige
 * @{
 ******************************************************************************/
#include "msp430.h"
#include "HAL_SDCard.h"
#include "driverlib.h"

//Pins from MSP430 connected to the SD Card
#define SPI_SIMO        BIT2
#define SPI_SOMI        BIT3
#define SPI_CLK         BIT1
#define SD_CS           BIT7

//Ports
#define SD_SPI_SEL      P9SEL
#define SD_SPI_DIR      P9DIR
#define SD_CS_SEL       P8SEL
#define SD_CS_OUT       P8OUT
#define SD_CS_DIR       P8DIR

#define USCI_SD_BASE    USCI_A2_BASE

//KLQ
#define SD_SPI_REN         P9REN
#define SD_SPI_OUT         P9OUT
//KLQ

/***************************************************************************//**
 * @brief   Initialize SD Card
 * @param   None
 * @return  None
 ******************************************************************************/
void SDCard_init (void)
{

    //Initialize USCI_A2 for SPI Master operation

        USCI_A_SPI_initMasterParam paramSD = {0};       // structure for USCI-SPI configuration
        paramSD.selectClockSource = USCI_A_SPI_CLOCKSOURCE_ACLK;  // SMCLK as clock source
        paramSD.clockSourceFrequency = 20000000;        // frequency of the SMCLK   20 MHz
        paramSD.desiredSpiClock = 400000;               // Required initial speed 400 kbps
        paramSD.msbFirst = USCI_A_SPI_MSB_FIRST;       // MSB first transfer
        paramSD.clockPhase = USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;   // phase select
        paramSD.clockPolarity = USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;  // Stay high when inactive
        //paramSD.spiMode = USCI_B_SPI_3PIN;     // 3-wire SPI
        USCI_A_SPI_initMaster(USCI_SD_BASE, &paramSD);    // EUSCI_B1 module

        //Enable SPI module
        USCI_A_SPI_enable(USCI_SD_BASE);      // Enable SPI
}

/***************************************************************************//**
 * @brief   Enable fast SD Card SPI transfers. This function is typically
 *          called after the initial SD Card setup is done to maximize
 *          transfer speed.
 * @param   None
 * @return  None
 ******************************************************************************/
void SDCard_fastMode (void)
{
    UCA2CTL1 |= UCSWRST;                                    //Put state machine in reset
    UCA2BR0 = 1;                                            //f_UCxCLK = 25MHz/2 = 12.5MHz
    UCA2BR1 = 0;
    UCA2CTL1 &= ~UCSWRST;                                   //Release USCI state machine
}

/***************************************************************************//**
 * @brief   Read a frame of bytes via SPI
 * @param   pBuffer Place to store the received bytes
 * @param   size Indicator of how many bytes to receive
 * @return  None
 ******************************************************************************/
void SDCard_readFrame (uint8_t *pBuffer, uint16_t size)
{
    uint16_t gie = __get_SR_register() & GIE;               //Store current GIE state

    __disable_interrupt();                                  //Make this operation atomic

    UCA2IFG &= ~UCRXIFG;                                    //Ensure RXIFG is clear

    //Clock the actual data transfer and receive the bytes
    while (size--){
        while (!(UCA2IFG & UCTXIFG)) ;                      //Wait while not ready for TX
        UCA2TXBUF = 0xff;                                   //Write dummy byte
        while (!(UCA2IFG & UCRXIFG)) ;                      //Wait for RX buffer (full)
        *pBuffer++ = UCA2RXBUF;
    }

    __bis_SR_register(gie);                                 //Restore original GIE state
}

/***************************************************************************//**
 * @brief   Send a frame of bytes via SPI
 * @param   pBuffer Place that holds the bytes to send
 * @param   size Indicator of how many bytes to send
 * @return  None
 ******************************************************************************/
void SDCard_sendFrame (uint8_t *pBuffer, uint16_t size)
{




    //Clock the actual data transfer and send the bytes. Note that we
    //intentionally not read out the receive buffer during frame transmission
    //in order to optimize transfer speed, however we need to take care of the
    //resulting overrun condition.
#ifndef WITH_DMA
    uint16_t gie = __get_SR_register() & GIE;               //Store current GIE state
  //  if(size>1000){
    __disable_interrupt();                                  //Make this operation atomic
    while (size--){
        while (!(UCA2IFG & UCTXIFG)) ;                      //Wait while not ready for TX
        UCA2TXBUF = *pBuffer++;                             //Write byte
    }
    while (UCA2STAT & UCBUSY) ;                             //Wait for all TX/RX to finish

    UCA2RXBUF;                                              //Dummy read to empty RX buffer
 //  }                                                 //and clear any overrun conditions
    __bis_SR_register(gie);                                 //Restore original GIE state
#else

    //uint16_t gie = __get_SR_register() & GIE;
        DMA2CTL &= ~DMAEN;
          DMA2SA = (uint32_t) pBuffer;
          //__disable_interrupt();
    //      DMA_setSrcAddress(DMA_CHANNEL_2, (uint32_t)pBuffer, DMA_DIRECTION_INCREMENT);
          DMA2SZ = size;
          //__delay_cycles(10);

          /* Configure the DMA transfer*/
          DMA2CTL =//DMAREQ | DMAEN;
            DMAREQ  |                           /* start transfer */
            DMADT_1 |//DMADT_0 |                           /* Single transfer mode */
            DMASBDB |                           /* Byte mode */
            DMAEN |                             /* Enable DMA */
            DMALEVEL |                          /* Level trigger */
            DMASRCINCR1 | DMASRCINCR0;          /* Increment the source address */
          //
          //_nop();
          //_nop();
          //_nop();
          //_nop();
          //__delay_cycles(10);
          while (UCA2STAT & UCBUSY) ;
          //__bis_SR_register(gie);
          //UCA2RXBUF;

#endif

}

/***************************************************************************//**
 * @brief   Set the SD Card's chip-select signal to high
 * @param   None
 * @return  None
 ******************************************************************************/
void SDCard_setCSHigh (void)
{
    SD_CS_OUT |= SD_CS;

}

/***************************************************************************//**
 * @brief   Set the SD Card's chip-select signal to low
 * @param   None
 * @return  None
 ******************************************************************************/
void SDCard_setCSLow (void)
{
    SD_CS_OUT &= ~SD_CS;

}

/***************************************************************************//**
 * @}
 ******************************************************************************/
//Released_Version_5_20_06_02
