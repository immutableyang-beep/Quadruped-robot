/*
 * File:   Q_BOT_header.c
 * Author: Yang
 */


#include "xc.h"
#include "FinalProject_header.h"

void Basic_setup(void) {
    CLKDIVbits.RCDIV = 0;
    AD1PCFG = 0x9fff;
    
    TRISBbits.TRISB8 = 1; // SCL
    TRISBbits.TRISB9 = 1; // SDA
////////////////////////////////////////////////////////////button setup
    TRISBbits.TRISB15 = 1;
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB13 = 1;
    TRISBbits.TRISB12 = 1;
    
    CNPU1bits.CN15PUE  = 1;  // RB11
    CNPU1bits.CN14PUE = 1;   // RB12
    CNPU1bits.CN13PUE = 1;   // RB13
    CNPU1bits.CN12PUE  = 1;  // RB14
    CNPU1bits.CN11PUE  = 1;  // RB15
    I2C1CONbits.I2CEN = 0;
    I2C1BRG = 79; // 100kHz
    I2C1STAT = 0;
    I2C1CONbits.DISSLW = 1;
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.I2CEN = 1;      
}

void Timer1_init(void) {
    T1CON = 0;           
    TMR1 = 0;           
    PR1 = WALK_SPEED;     
    T1CONbits.TCKPS = 0b11;  
    IFS0bits.T1IF = 0;   
    IEC0bits.T1IE = 1;   
    T1CONbits.TON = 1;
}// 400ms delay(gait time gap)

void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 1600; j++) {
            Nop();  
        }
    }
}

void PCA_write(unsigned char reg, unsigned char data) {
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN);  //start
    IFS1bits.MI2C1IF = 0; 
        
    I2C1TRN = 0x80; 
    while (I2C1STATbits.TRSTAT); //address
    while (I2C1STATbits.ACKSTAT);
    IFS1bits.MI2C1IF = 0; 
        
    I2C1TRN = reg;  //Register select
    while (I2C1STATbits.TRSTAT);
    while (I2C1STATbits.ACKSTAT);
    IFS1bits.MI2C1IF = 0; 
        
    I2C1TRN = data; // Data input
    while (I2C1STATbits.TRSTAT);
    while (I2C1STATbits.ACKSTAT);
    IFS1bits.MI2C1IF = 0; 
    
    
    I2C1CONbits.PEN = 1;    //stop     
    while (I2C1CONbits.PEN);
    IFS1bits.MI2C1IF = 0; 
}

void PCA_init() {
    // 1) Sleep
    PCA_write(0x00, 0x10);     // MODE1: SLEEP=1

    // 2) Set PWM frequency (50Hz)
    PCA_write(0xFE, 121);      // PRE_SCALE=121

    // 3) Wake up and enable Auto-Increment
    PCA_write(0x00, 0x20);     // MODE1: AI=1, SLEEP=0

    // 4) Optional: set output structure to totem pole (better for servos)
    PCA_write(0x01, 0x04);     // MODE2: OUTDRV=1
}

int readAngle (int servoAngle) {
    int angleVal;
    angleVal = SERVO_MID + (servoAngle * (SERVOSPAN / 180));
    return angleVal;
}

void setServoAngle (int selServo,int setAngle) {
    uint8_t on_low = 0x00;
    uint8_t on_high = 0x00;
    uint8_t off_low = (readAngle(setAngle) & 0xFF);
    uint8_t off_high = (readAngle(setAngle) >> 8);
    
    switch(selServo) {
        case 1:
            PCA_write(0x06, on_low);
            PCA_write(0x07, on_high);
            PCA_write(0x08, off_low);
            PCA_write(0x09, off_high);
            break;
        case 2:
            PCA_write(0x0A, on_low);
            PCA_write(0x0B, on_high);
            PCA_write(0x0C, off_low);
            PCA_write(0x0D, off_high);
            break;
        case 3:
            PCA_write(0x0E, on_low);
            PCA_write(0x0F, on_high);
            PCA_write(0x10, off_low);
            PCA_write(0x11, off_high);
            break;
        case 4:
            PCA_write(0x12, on_low);
            PCA_write(0x13, on_high);
            PCA_write(0x14, off_low);
            PCA_write(0x15, off_high);
            break;
        case 5:
            PCA_write(0x16, on_low);
            PCA_write(0x17, on_high);
            PCA_write(0x18, off_low);
            PCA_write(0x19, off_high);
            break;
        case 6:
            PCA_write(0x1A, on_low);
            PCA_write(0x1B, on_high);
            PCA_write(0x1C, off_low);
            PCA_write(0x1D, off_high);
            break;
        case 7:
            PCA_write(0x1E, on_low);
            PCA_write(0x1F, on_high);
            PCA_write(0x20, off_low);
            PCA_write(0x21, off_high);
            break;
        case 8:
            PCA_write(0x22, on_low);
            PCA_write(0x23, on_high);
            PCA_write(0x24, off_low);
            PCA_write(0x25, off_high);
            break;
    }
}

void initAction (void) {
    setServoAngle(1,60);
    setServoAngle(2,-35);
    setServoAngle(3,90);
    setServoAngle(4,-60);
        
    setServoAngle(5,-140);
    setServoAngle(6,120);
    setServoAngle(7,120);
    setServoAngle(8,-200);
}

////////////////////////////////////////////////////////////////////////////////GAIT CONTROL

volatile int MoveOrNot = 0;

void __attribute__((interrupt, auto_psv)) _T1Interrupt() {
    IFS0bits.T1IF = 0; 
    MoveOrNot = 1;
}

int (*current_step)[4]; // allocate and find the state and steps
int step_index = 0;

int step_delay[8] = {1, 1, 1, 0, 1, 1, 1, 0};  // 1 means this step should wait 400ms to excute next step

int step_forward [8][4] = { //servo A.angle A,servo B, angle B
    {5,-100,7,80},    // {5,-80,7,60}
    {1,160,3,-40},
    {5,-140,7,120},
    {1,60,3,90},
    {6,80,8,-160},    // {6,60,8,-140}
    {2,-95,4,70},
    {6,130,8,-210},
    {2,-35,4,-60}
};
int step_back [8][4] = {
    {5,-100,7,80},    
    {1,0,3,160},
    {5,-140,7,120},
    {1,60,3,90},
    {6,80,8,-160},
    {2,25,4,-130},
    {6,130,8,-210},
    {2,-35,4,-60}   
};

int step_left [8][4] = {
    {5,-100,7,80},    
    {1,160,3,160},
    {5,-140,7,120},
    {1,60,3,90},
    {6,80,8,-160},
    {2,25,4,70},
    {6,130,8,-210},
    {2,-35,4,-60}  
};
int step_right [8][4] = {
    {5,-100,7,80},    
    {1,0,3,-40},
    {5,-140,7,120},
    {1,60,3,90},
    {6,80,8,-160},
    {2,-95,4,-130},
    {6,130,8,-210},
    {2,-35,4,-60}     
};

void moveStraight (int step) {
    current_step = step_forward;
    step_index = 0; 
    for (int i = 0; i < step; i++) {
        if (step_delay[step_index] == 1) {
            while (!MoveOrNot);
            MoveOrNot = 0;
        }     
        else {
            MoveOrNot = 1;
        }
        int S_1 = current_step[step_index][0];
        int A_1 = current_step[step_index][1];
        int S_2 = current_step[step_index][2];
        int A_2 = current_step[step_index][3];
        setServoAngle(S_1,A_1);
        setServoAngle(S_2,A_2);
        step_index++;
        
        if (step_index >= CURRENT_STEP_COUNT) {
            step_index = 0;
        }
    }   
}

void turnLeft (int step) {
    current_step = step_left;
    step_index = 0; 
    for (int i = 0; i < step; i++) {
        if (step_delay[step_index] == 1) {
            while (!MoveOrNot);
            MoveOrNot = 0;
        }
        else {
            MoveOrNot = 1;
        }
        int S_1 = current_step[step_index][0];
        int A_1 = current_step[step_index][1];
        int S_2 = current_step[step_index][2];
        int A_2 = current_step[step_index][3];
        setServoAngle(S_1,A_1);
        setServoAngle(S_2,A_2);
        step_index++;
           
        if (step_index >= CURRENT_STEP_COUNT) {
            step_index = 0;
        }
    }
}

void turnRight(int step) {
    current_step = step_right;
    step_index = 0; 
    for (int i = 0; i < step; i++) {
        if (step_delay[step_index] == 1) {
            while (!MoveOrNot);
            MoveOrNot = 0;
        }
        else {
            MoveOrNot = 1;
        }
        int S_1 = current_step[step_index][0];
        int A_1 = current_step[step_index][1];
        int S_2 = current_step[step_index][2];
        int A_2 = current_step[step_index][3];
        setServoAngle(S_1,A_1);
        setServoAngle(S_2,A_2);
        step_index++;
        
        if (step_index >= CURRENT_STEP_COUNT) {
            step_index = 0;
        }
    }    
}

void moveBack(int step) {
    current_step = step_back;
    step_index = 0; 
    for (int i = 0; i < step; i++) {
        if (step_delay[step_index] == 1) {
            while (!MoveOrNot);
            MoveOrNot = 0;
        }
        else {
            MoveOrNot = 1;
        }
        int S_1 = current_step[step_index][0];
        int A_1 = current_step[step_index][1];
        int S_2 = current_step[step_index][2];
        int A_2 = current_step[step_index][3];
        setServoAngle(S_1,A_1);
        setServoAngle(S_2,A_2);
        step_index++;
        
        if (step_index >= CURRENT_STEP_COUNT) {
            step_index = 0;
        }
    }   
}

int step_opt (int step) {
    return step * 8;
} 


void speed(void) {
    if (PR1 == RUN_SPEED) {
        PR1 = WALK_SPEED;
    }
    else {
        PR1 = RUN_SPEED;
    }

}
