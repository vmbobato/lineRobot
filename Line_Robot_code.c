#include "msp.h"

void slowRight(void); //for soft turns, stop 1 motor, but let other keep spinning (rn its a hard turn for both soft and hard)
void slowLeft(void);
void fastRight(void); //for hard turns, turn motors in opposite directions
void fastLeft(void);
void forward(void);
void reverse(void);
void stop(void);
void checkPos(void);
void buzzer(void);
void openServo(void);
void closeServo(void);
void secondPos(void);

//ultrasonic func
void TA2_N_IRQHandler(void);

//variables
unsigned int rightSensor;
unsigned int middleSensor;
unsigned int leftSensor;
int delay;
volatile unsigned int tcap=0, tcap_flag=0, tcap_cov=0;
int usCount = 0;
int checkConsistant;
int check;
void main(void)

{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    P6->DIR &= 0xFC; 

    P2->DIR |= BIT4 | BIT5 | BIT6 | BIT7; 
    P2->SEL1 &= ~(BIT4 | BIT5 | BIT6 | BIT7);
    P2->SEL0 |= (BIT4 | BIT5 | BIT6 | BIT7); 
    P5->DIR |= 0x01; 
    P5->OUT |= BIT0;
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_CLR;

    P5->DIR |= BIT6;
    P5->SEL1 &= ~BIT6;
    P5->SEL0 |= BIT6;
    TIMER_A2->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_CLR;
    closeServo();

    ADC14->CTL0 = ADC14_CTL0_SHT0_6 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_3;
    ADC14->CTL0 |= ADC14_CTL0_MSC | ADC14_CTL0_CONSEQ_1;
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_11;
    ADC14->MCTL[1] = ADC14_MCTLN_INCH_12;
    ADC14->MCTL[2] = ADC14_MCTLN_INCH_13 | ADC14_MCTLN_EOS;

    P3->DIR |= BIT6;
    TIMER_A1->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_CLR;

    while(1){
        if((P6->IN & BIT0) == 0){ 
            int i;
            for(i=0;i<=2;i++){
                delay = 10000;
                while(delay >= 0){
                    buzzer();
                    delay--;
                }
                __delay_cycles(200000);
            }

            for(i=0;i<=2;i++){
                openServo();
                __delay_cycles(400000);
                closeServo();
                __delay_cycles(400000);
            }
            break;
        }
    }

    P5->DIR &= ~BIT6; 
    P3->DIR |= BIT5;
    P5->DIR &= ~BIT7;
    P5->SEL0 |= BIT7;
    P5->SEL1 &= ~BIT7;
    TIMER_A2->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_2 | TIMER_A_CTL_CLR;
    TIMER_A2->CCTL[2] = TIMER_A_CCTLN_CM_2 | TIMER_A_CCTLN_CCIS_0 | TIMER_A_CCTLN_CCIE | TIMER_A_CCTLN_CAP | TIMER_A_CCTLN_SCS;
    TIMER_A2->CTL |= TIMER_A_CTL_IE;
    NVIC->ISER[0] = 1 << ((TA2_N_IRQn) & 31);
    __enable_irq();

    while(1){

        checkPos();

        if((P6->IN & BIT1) == 0){ 
            stop();
            break;
        }

        tcap_flag = 0;
        tcap_cov = 0;
        TIMER_A2->CTL |= TIMER_A_CTL_CLR;
        P3->OUT |= BIT5;
        __delay_cycles(30);
        P3->OUT &= ~BIT5;
        __delay_cycles(100);
        while(tcap_flag==0);

        if(tcap > 4000){
            checkConsistant = 0;
        }
        if(tcap <= 3000){
            usCount++;
            checkConsistant = 1;
            if(usCount == 10 & checkConsistant == 1){
                stop();
                secondPos();
                break;
            }
        }
        __delay_cycles(10000);
    }

    while(1){
        checkPos();
        if((P6->IN & BIT1) == 0){ //down pressed
            stop();
            break;
        }
    }
}


void slowRight(void){
    //M1
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[1] = 107; //set val of CCR1
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 5000; //set val of CCR2
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    //M2
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[3] = 214; //set val of CCR1
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = 214; //set val of CCR2
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
}

void slowLeft(void){
    //M1
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[1] = 214; //set val of CCR1
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 214; //set val of CCR2
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    //M2
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[3] = 107; //set val of CCR1
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = 5000; //set val of CCR2
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
}

void fastRight(void){
    //M1
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[1] = 5000; //set val of CCR1
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 107; //set val of CCR2
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    //M2
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[3] = 107; //set val of CCR1
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = 5000; //set val of CCR2
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
}
void fastLeft(void){
    //M1
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[1] = 107; //set val of CCR1
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 5000; //set val of CCR2
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    //M2
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[3] = 5000; //set val of CCR1
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = 107; //set val of CCR2
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
}
void forward(void){
    //M1
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[1] = 107; //set val of CCR1
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 5000; //set val of CCR2
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    //M2
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[3] = 107; //set val of CCR1
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = 5000; //set val of CCR2
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
}
void reverse(void){
    //M1
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[1] = 5000; //set val of CCR1
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 107; //set val of CCR2
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    //M2
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[3] = 5000; //set val of CCR1
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = 107; //set val of CCR2
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
}
void stop(void){
    //M1
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[1] = 214; //set val of CCR1
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 214; //set val of CCR2
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    //M2
    TIMER_A0->CCR[0] = 214; //set val of CCR0
    TIMER_A0->CCR[3] = 214; //set val of CCR1
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = 214; //set val of CCR2
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
}

void checkPos(void){
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
    while((ADC14->CTL0 & ADC14_CTL0_BUSY) != 0);
    leftSensor = ADC14->MEM[0];
    middleSensor = ADC14->MEM[1]; //gets middle value
    rightSensor = ADC14->MEM[2];

    if(rightSensor < 12000 & middleSensor > 12000 & leftSensor < 12000){ //outer sensoslowRight over white
        forward();
    }else if(rightSensor > 12000 & middleSensor > 12000 & leftSensor < 12000){ //left sensor over white only
        slowLeft();
    }else if(rightSensor < 12000 & middleSensor > 12000 & leftSensor > 12000){ //right sensor over white only
        slowRight();
    }else if(rightSensor < 12000 & middleSensor < 12000 & leftSensor > 12000){ //right & middle sensor over white
        fastLeft();
    }else if(rightSensor > 12000 & middleSensor < 12000 & leftSensor < 12000){ //middle & left sensor over white
        fastRight();
    }else if(rightSensor < 12000 & middleSensor < 12000 & leftSensor < 12000){ //all sensor over white
        reverse();
    }
}

void buzzer(void){
    TIMER_A1->CCR[0] = 6000;
    if((TIMER_A1->CCTL[0] & TIMER_A_CCTLN_CCIFG) != 0){
        P3->OUT ^= BIT6;
        TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    }
}

void openServo(void){
    TIMER_A2->CCR[0]=60000;
    TIMER_A2->CCTL[1]=TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A2->CCR[1]=4500;
}

void closeServo(void){
    TIMER_A2->CCR[0]=60000;
    TIMER_A2->CCTL[1]=TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A2->CCR[1]=5500;
}

void secondPos(void){
    P5->DIR |= BIT7; //disable ultrasonic
    P5->DIR |= BIT6; //enable servo
    TIMER_A2->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_CLR;
    //open and close 3 times
    openServo();
    __delay_cycles(400000);
    closeServo();
    __delay_cycles(400000);
    openServo();
    __delay_cycles(400000);
    closeServo();
    __delay_cycles(400000);
    openServo();
    __delay_cycles(400000);
    closeServo();
    __delay_cycles(400000);
    openServo();
    __delay_cycles(400000);
    fastRight();
    __delay_cycles(3555000); 
    stop();
}

void TA2_N_IRQHandler(void){
    if((TIMER_A2->CTL & TIMER_A_CTL_IFG)!=0){
        tcap_cov = 1;
        tcap_flag = 1;
        TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
    }
    if((TIMER_A2->CCTL[2] & TIMER_A_CCTLN_CCIFG)!=0){
        tcap = TIMER_A2->CCR[2];
        tcap_flag = 1;
        TIMER_A2->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;
    }
}
