#include <utils.h>

void utilsConsumeNonNumeric()
{
	int c;

	do {
		c = getchar();
	}while ((!isdigit(c)) || c == (int) '-');

	ungetc(c, stdin);
}

void utilsGetOption(char * firstOptionDesc, ...)
{
	int var_int;
	float var_float;
	va_list args;
	char * optionDesc;
	int optionType;
	void * optionVar;

	va_start(args, firstOptionDesc);

	optionDesc = firstOptionDesc;

	do{
		trace_printf(optionDesc);

		optionType = va_arg(args, int);
		optionVar = va_arg(args, void *);

		switch(optionType)
		{
			case UTILS_OPTION_TYPE_UINT8:
				while (scanf("%d", &var_int) == 0)
					utilsConsumeNonNumeric();

				*((uint8_t *) optionVar) = (uint8_t) var_int;
				break;
			case UTILS_OPTION_TYPE_INT8:
				while (scanf("%d", &var_int) == 0)
					utilsConsumeNonNumeric();

				*((int8_t *) optionVar) = (int8_t) var_int;
				break;
			case UTILS_OPTION_TYPE_FLOAT32:
				while (scanf("%f", &var_float) == 0)
					utilsConsumeNonNumeric();

				*((float32_t *) optionVar) = (float32_t) var_float;
				break;
			default:
				break;

		}

		optionDesc = va_arg(args, char *);

		if(optionDesc == NULL)
			break;
	} while(1);

	va_end(args);
}
