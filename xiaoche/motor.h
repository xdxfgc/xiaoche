#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_set_speed(int32_t left, int32_t right);
void motor_stop(void);

#endif /* MOTOR_H */
