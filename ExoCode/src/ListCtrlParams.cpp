#include "ListCtrlParams.h"

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

void ctrl_param_array_gen() {
	Serial.print("\n**(^*&^*&^&Running ListCtrlParams.cpp$%^&#$#*&(");
	
	//Begin SD card
	if (!SD.begin(SD_SELECT)) {
			while (1)
			
			if (Serial)
			{
				// logger::print("SD.begin() failed");
				// logger::print("\n");
				Serial.println("SD.begin() failed");
			}
	}
	
	//Configure
	static char stringArray[(uint8_t)config_defs::ankle_controllers::Count][MAX_COLUMNS][MAX_STRING_LENGTH];
	
	Serial.print("  |  Number of ankle controllers: ");
	Serial.print((uint8_t)config_defs::ankle_controllers::Count);
	
	uint8_t row_idx = 0;
	for (int i_ctrl = 2; i_ctrl < (uint8_t)config_defs::ankle_controllers::Count; i_ctrl++) {
        Serial.print("Current index value: ");
        Serial.println(i_ctrl);
		
        if (controller_parameter_filenames::ankle.count(i_ctrl)) { // condition is true if count is 1
			Serial.print("Key ");
			Serial.print((int)i_ctrl);
			Serial.println(" exists.");
			
			std::string filename = controller_parameter_filenames::ankle[i_ctrl];
			Serial.print("\nstd::string filename is ");
			Serial.print(filename.c_str());
			Serial.print("char filename_char = filename.c_str() returns ");
			const char* filename_char = filename.c_str();
			Serial.print(filename_char);
			
			
			// Call the function to read and parse the fifth row
		int columnsRead = readAndParseFifthRow(filename_char, stringArray, MAX_COLUMNS, MAX_STRING_LENGTH, row_idx);
		

		// Print the results
		if (columnsRead > 0) {
			Serial.println("\nFifth Row Data Saved:");
			for (int i = 0; i < columnsRead; i++) {
			  Serial.print("Column ");
			  Serial.print(i + 1);
			  Serial.print(": ");
			  Serial.println(stringArray[row_idx][i]);
			}
		}
		else {
			Serial.println("\nFailed to read or parse the fifth row.");
		}
		
		row_idx++;
	}

}
	
	
	
	
	
}

// --- Function to Read and Parse ---

 int readAndParseFifthRow(const char* filename_char, char arr[][MAX_COLUMNS][MAX_STRING_LENGTH], int maxCols, int maxLen, uint8_t row_idx) {
	 Serial.print("\nIncoming filename_char is ");
	 Serial.print(filename_char);
  File dataFile = SD.open(filename_char);
  if (!dataFile) {
    Serial.print("Error opening ");
    Serial.println(filename_char);
    return 0; // Return 0 columns read
  }

  // 1. Find the Fifth Line
  int rowCount = 0;
  String targetLine = "";
  
  // Read byte by byte until the end of the file or the fifth row is found
  while (dataFile.available()) {
    char c = dataFile.read();
    
    // Append character to the current line string
    if (rowCount == 4) { // Row 5 is index 4 (0-based)
      targetLine += c;
    }
    
    // Check for a newline character to count rows
    if (c == '\n') {
      rowCount++;
      if (rowCount > 4) {
        break; // Stop after reading the entire fifth row
      }
    }
  }

  dataFile.close(); // Always close the file!

  if (rowCount < 4) {
    Serial.println("File is too short (less than 5 rows).");
    return 0;
  }
  
  // 2. Parse the Line (Tokenize the String)
  int colIndex = 0;
  int charIndex = 0;
  
  for (int i = 0; i < targetLine.length(); i++) {
    char c = targetLine.charAt(i);

    if (c == ',') {
      // End of a field: terminate the current string and move to the next column
      arr[row_idx][colIndex][charIndex] = '\0'; // Null-terminate the string
      colIndex++;
      charIndex = 0; // Reset character index for the next column
      
      // Safety check: stop if max columns reached
      if (colIndex >= maxCols) break;

    }
	else if (c != '\r' && c != '\n') { // Ignore carriage return and newline characters
      // Append character to the current string
      if (charIndex < maxLen - 1) { // -1 for the null terminator
        arr[row_idx][colIndex][charIndex] = c;
        charIndex++;
      } else {
        Serial.println("Warning: String truncated.");
      }
    }
  }
  
  // 3. Handle the Last Column
  // Null-terminate the last string after the loop finishes
  if (colIndex < maxCols && charIndex > 0) {
    arr[row_idx][colIndex][charIndex] = '\0';
    colIndex++; // Increment to count the last column
  }

	Serial.print("\nNumber of columns read: ");
	Serial.print(colIndex);

  return colIndex; // Return the number of columns successfully read
}

#endif