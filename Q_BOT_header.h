 #ifndef FINALPROJECT_HEADER_H
#define	FINALPROJECT_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.

#define SERVOSPAN 205
#define SERVO_MID 307

#define CURRENT_STEP_COUNT 8

#define WALK_SPEED 31249 // 400 ms
#define RUN_SPEED 15624  // 200 ms

void Basic_setup(void);
void Timer1_init(void);
void delay_ms(unsigned int ms);
void PCA_write(unsigned char reg, unsigned char data);
void PCA_init();

//////////////////////////////////////////////////////////////// GAIT SIMULATION
void initAction(void);
int readAngle (int servoAngle);
void setServoAngle (int selServo,int setAngle);
void moveStraight (int step);
void turnLeft (int step);
void turnRight (int step);
void moveBack(int step);
int step_opt (int step);
//////////////////////////////////////////////////////////////// GAIT SIMULATION
void speed(void);
#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

