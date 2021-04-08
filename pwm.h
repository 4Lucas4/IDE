#ifndef PWM_H_
#define PWM_H_

void FTM0_init(void);
void FTM0_C2C3_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int dir);
void FTM0_C0C1_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int dir);
void FTM3_init(void);
void FTM3_set_duty_cycle(unsigned int duty_cycle);
void LeftWheel(signed int speed);
void RightWheel(signed int speed);
void Forward(unsigned int speed);
void Reverse(unsigned int speed);
void Stop(void);
void Steer(unsigned int amount);
void Straight(void);
void Left(unsigned int amount);
void Right(unsigned int amount);

#endif /* PWM_H_ */
