#ifndef UTILS_H_
#define UTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdint.h>
#include <arm_math.h>
#include <diag/Trace.h>

typedef enum UtilsOptionType {
	UTILS_OPTION_TYPE_UINT8 = 0,
	UTILS_OPTION_TYPE_INT8,
	UTILS_OPTION_TYPE_FLOAT32
} UtilsOptionTypeDef;

void utilsConsumeNonNumeric();
void utilsGetOption(char * firstOptionDesc, ...);

#endif /* UTILS_H_ */
