/*
 * utils.h
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */


#ifndef UTILS_H_
#define UTILS_H_
#include "configuration.h"

char *xstrtok(char **m,char *s,char c);
bool isLetter(char);
bool isNumber(char);
void getValues(char*, KeyPair_t*, uint8_t);

#endif /* UTILS_H_ */
