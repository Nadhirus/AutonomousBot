#ifndef WALLFOLLOWER_H
#define WALLFOLLOWER_H
#include <stdint.h>

void wallfollower_init(void);
void wallfollower_update(void); // calls diffdrive_set_speed() internally

#endif
