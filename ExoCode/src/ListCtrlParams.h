#ifndef ListCtrlParams_h
#define ListCtrlParams_h



#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "ExoData.h"
#include "ParseIni.h"
#include "Utilities.h"
#include "ParamsFromSD.h"

#include <SPI.h>
#include <SD.h>
#include <map>
#include <string>

#ifndef SD_SELECT
    #define SD_SELECT BUILTIN_SDCARD
#endif

	// Define the size of the array and the max length for each string
	const int MAX_COLUMNS = 22;
	const int MAX_STRING_LENGTH = 30;
	

	// Array to hold the strings from the fifth row

void ctrl_param_array_gen();
int readAndParseFifthRow(const char* filename_char, char arr[][MAX_COLUMNS][MAX_STRING_LENGTH], int maxCols, int maxLen, uint8_t row_idx);


#endif
#endif