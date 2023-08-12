#include "msp.h"
void initPort(void);
void initTimer(void);
void enable25MHz(void);
void turnOnCheck();
void turnOn();
void turnOff();
void pixelOut(uint8_t*);
/**
 * main.c
 */

void initPort(void){

    /*Configure RED LED1 P1.0 as output. Setup Switch S1 (P1.1) and Switch S2 (P1.4) as interrupt buttons*/
    P1->SEL0 = 0;
    P1->SEL1 = 0;
    P1->DIR = 0x01; //0000 0001 0 in 1 out
    P1->OUT = 0x12; //0001 0010
    P1->REN = 0x12; // Enable pull-up resistor for inputs
    /*Interrupt for Port 1*/
      P1->IES = 0x12; //Set P1.1 S1 and S1.4 S2 as interrupt switch
      P1->IFG &= ~0x12;//Lower the interrupt flag
      P1->IE = 0x12; //Enable interrupt pin
      NVIC->IP[8] = (NVIC->IP[8]&0x00FFFFFF)|0x80000000; //Set Port 1's interrupt priority to 4 (#35 Interrupt)
      NVIC->ISER[1] = 0x00000008; //Enable Port1 Interrupt

    /*Configure P2.7, 2.6 as output port. P2.7 is hsync, P2.6 is vsync*/
    P2->SEL0 = 0x00;   //0000 0000 LED2 and P2.7, P2.6 setup as GPIO
    P2->SEL1 = 0x00;   //0000 0000
    P2->DIR = BIT0 | BIT1 | BIT2 | BIT6 | BIT7;
    P2->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT6 | BIT7);

    /*Individual RGB pin*/
    P4->SEL0 = 0; //P4 is RGB output
    P4->SEL1 = 0;
    P4->DIR = 0x3F;//0011 1111
    P4->OUT &= ~0x3F;



}
uint16_t CCRVal = 630;
void initTimer(void){
    /*Timer A0: setup to control horizontal timing*/
    TIMER_A0->CCR[0] = 0x31A; //794th line, swing to left and increment vertical
    TIMER_A0->CCR[1] = 635; //635th line, start front porch
    TIMER_A0->CCR[2] = 651; //651th line, start hsync
    TIMER_A0->CCR[3] = 746; //746th line, start back porch
    /*Timer A1: setup to control the first and second pixel blocks*/
    TIMER_A1->CCR[0] = 0x31A;
    TIMER_A1->CCR[1] = CCRVal;



    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK  | // SMCLK, up mode
                    TIMER_A_CTL_MC__UP;
    TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK  | // SMCLK, up mode
                    TIMER_A_CTL_MC__UP;

    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Enable sleep on exit from ISR

    /*Interrupt for TimerA0 and TimerA1*/

    NVIC->ISER[0] |= 1 << ((TA0_0_IRQn) & 31);
    NVIC->ISER[0] |= 1 << ((TA0_N_IRQn) & 31);
    NVIC->ISER[0] |= 1 << ((TA1_0_IRQn) & 31);
    NVIC->ISER[0] |= 1 << ((TA1_N_IRQn) & 31);


    // TIMER_A0->CCR[0] toggle, interrupt enabled
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;
    // TA0CCR1 toggle, interrupt enabled
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;
    // TA0CCR2 toggle, interrupt enabled
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_CCIE;
    // TA0CCR3 toggle, interrupt enabled
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_CCIE;
    // TA1CCR1 toggle, interrupt enabled
    TIMER_A1->CCTL[1] = TIMER_A_CCTLN_CCIE;

}
void enable25MHz(void){
    CS->KEY = CS_KEY_VAL;
    CS->CTL1 = (CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_SELS_MASK | CS_CTL1_SELA_MASK))//Reset all values to 0
            | CS_CTL1_SELM__MODOSC //MCLK uses MODOSC, 25Mhz
            | CS_CTL1_SELS__MODOSC //SMCLK uses MODOSC
            | CS_CTL1_SELA__REFOCLK ;// ACLK uses REFOCLK
    CS->CLKEN |= CS_CLKEN_MODOSC_EN;//Ensure MODOSC is enabled.
    while( !(CS->STAT & CS_STAT_MODOSC_ON) ){
        volatile uint32_t i;
        P1->OUT ^= BIT0;    //if MODOSC is not ready, flash LED1 to show error
        for(i = 0; i < 25000000; i++);
    }
    CS->KEY = 0;

}
uint16_t verticalCount = 0x00;
uint8_t hdisplayReady = 1;
uint8_t vdisplayReady = 1;

uint8_t* addr_ptr;
uint8_t* pixelStartPointer = (uint8_t*)0x1CC0;
uint8_t* pixelEndPointer = (uint8_t*)0x1D00;//End pointer will not be displayed.
void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    enable25MHz();
    initPort();
    initTimer();

    P2->OUT |= (BIT6 | BIT7); //P2.6 is vsync, P2.7 is hsync
    addr_ptr = pixelStartPointer;//Ensure the current address pointer points to the start location

    __enable_irq();
    while(1){

    }

}


void TA0_0_IRQHandler(void){
    if(TIMER_A0->CCTL[0] & TIMER_A_CCTLN_CCIFG){//794th line, end front porch, go to next line

        hdisplayReady = 1;
        verticalCount++;//increment frame indicator
        if(verticalCount < 0x1E0){//If less than 480, it is in visible area
            turnOnCheck();

        }
        else if(verticalCount == 0x1E0){//At 480th frame, outside of visible area, turn off display, start front porch
            vdisplayReady = 0;
            turnOnCheck();
        }
        else if(verticalCount == 0x1EA){//At 490th frame, lower vertical sync signal, end front porch, start blinking
            P2->OUT &= ~BIT6;
        }
        else if(verticalCount == 0x1EC){//At 492th frame, pull up vertical sync signal, end blinking, start back porch
            P2->OUT |= BIT6;
        }
        else if(verticalCount > 0x20D){//At 525th frame, reset vertical line count, end back porch, go back to visible area.
            verticalCount = 0x00;
            vdisplayReady = 1;
            turnOnCheck();
        }

        TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    }
}
void TA0_N_IRQHandler(void)
{
    if(TIMER_A0->CCTL[1]&TIMER_A_CCTLN_CCIFG){//After 635th line, outside of visible area, turn off display, start front porch
        hdisplayReady = 0;

        turnOnCheck();

        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;

        }
    if(TIMER_A0->CCTL[2]&TIMER_A_CCTLN_CCIFG){//After 651th line, end front porch, pull down the horizontal sync to blink

            P2->OUT &= ~BIT7;

            TIMER_A0->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;
    }
    if(TIMER_A0->CCTL[3]&TIMER_A_CCTLN_CCIFG){//After 746th line, pull up horizontal sync, start back porch

            P2->OUT |= BIT7;

            TIMER_A0->CCTL[3] &= ~TIMER_A_CCTLN_CCIFG;
        }
}
uint8_t subsection = 0x0001;
void TA1_N_IRQHandler(void)
{
    if(TIMER_A1->CCTL[1]&TIMER_A_CCTLN_CCIFG){
        if(TIMER_A1->CCR[1] < 630){//To display 2 pixels on the screen, CCR value is changed on the fly if less than 630.
            turnOff();
            pixelOut(addr_ptr + subsection);
            TIMER_A1->CCR[1] += CCRVal;
            subsection++;
        }else{//If reach 630th frame, reset the CCR Value
            pixelOut(addr_ptr);
            TIMER_A1->CCR[1] = CCRVal;
            subsection = 0x0001;
        }

        TIMER_A1->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;

    }

}

void PORT1_IRQHandler(void){
    turnOff();

    if(P1->IFG & BIT1){//If left button is pressed
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC_0; /* TimerA0 is halted */
        TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC_0; /* TimerA1 is halted */
        if(CCRVal == 630){//Change CCR value to show between 1 pixel and 2 pixels
            CCRVal = 315;
        }else if(CCRVal == 315){
            CCRVal = 630;
        }
        P1->OUT ^= BIT0;//Indicate display mode
        volatile uint32_t i;
        for(i = 0; i < 10000000; i++);//Button Bounce
        P1->IFG &= ~BIT1;//Lower Interrupt flag
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | 0x0004;/*TimerA0 reset, started, in up mode, use SMCLK */
        TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | 0x0004;/*TimerA1 reset, started, in up mode, use SMCLK */
    }else if(P1->IFG & BIT4){//If right button is pressed
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC_0; /* TimerA0 is halted */
        TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC_0; /* TimerA1 is halted */

        if(addr_ptr < pixelEndPointer){//Increment the address pointer to the next byte if in range
            addr_ptr += 0x0001;
        }
        else{//If outside of range, reset to start point
            addr_ptr = pixelStartPointer;
        }

        volatile uint32_t i;
        for(i = 0; i < 1000000; i++);//Button bounce
        P1->IFG &= ~BIT4;//Lower interrupt flag
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | 0x0004;/*TimerA0 reset, started, in up mode, use SMCLK */
        TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | 0x0004;/*TimerA1 reset, started, in up mode, use SMCLK */
    }

}
void turnOnCheck(){//Check whether horizontal and vertical syncs are ready
    if((hdisplayReady == 1) && (vdisplayReady == 1)){//Both ready, OK to make VGA output pins operational. Modify DIR instead of OUT so the current screen stays after blinking.
        P4->DIR |= 0x3F;
    }
    else{//Not ready, turn them off
        P4->DIR &= ~0x3F;
    }
}
void turnOn(){
    P4->OUT ^= 0x3F;

}
void turnOff(){//Turn off LED2 and VGA output
    P4->OUT &= ~0x3F;
    P2->OUT &= ~(BIT0|BIT1|BIT2);
}
void pixelOut(uint8_t* ptr){
    P2->OUT &= ~(BIT0|BIT1|BIT2);//Reset LED2
    if(ptr >= pixelEndPointer){//Check whether the pointer passed in is in range
        uint8_t diff = ptr - pixelEndPointer;
        ptr = diff + pixelStartPointer;
    }
    uint8_t rawDisplayData = *ptr;//Retrieve data from memory

    if(rawDisplayData & 0x3F){
        P4->OUT = (rawDisplayData & 0x3F);
        if(rawDisplayData & BIT0){
             //BIT0 of data controls Lower resistance bit of blue
            P2->OUT |= BIT2;
        }
        if(rawDisplayData & BIT1){
            //BIT1 of data controls Higher resistance bit of blue
            P2->OUT |= BIT2;
        }
        if(rawDisplayData & BIT2){
            //BIT2 of data controls Lower resistance bit of green
            P2->OUT |= BIT1;
        }
        if(rawDisplayData & BIT3){
            //BIT3 of data controls Higher resistance bit of green
            P2->OUT |= BIT1;
        }
        if(rawDisplayData & BIT4){
            //BIT4 of data controls Lower resistance bit of red
            P2->OUT |= BIT0;
        }
        if(rawDisplayData & BIT5){
             //BIT5 of data controls Higher resistance bit of red
            P2->OUT |= BIT0;
        }
    }
}
