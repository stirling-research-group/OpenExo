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
	char stringArray[MAX_COLUMNS][MAX_STRING_LENGTH];
	
	// Call the function to read and parse the fifth row
   int columnsRead = readAndParseFifthRow("/ankleControllers/spv2.csv", stringArray, MAX_COLUMNS, MAX_STRING_LENGTH);

  // Print the results
  if (columnsRead > 0) {
    Serial.println("\nFifth Row Data Saved:");
    for (int i = 0; i < columnsRead; i++) {
      Serial.print("Column ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(stringArray[i]);
    }
  } else {
    Serial.println("\nFailed to read or parse the fifth row.");
  } 
	
}

// --- Function to Read and Parse ---

 int readAndParseFifthRow(const char* filename, char arr[][MAX_STRING_LENGTH], int maxCols, int maxLen) {
  File dataFile = SD.open(filename);
  if (!dataFile) {
    Serial.print("Error opening ");
    Serial.println(filename);
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
      arr[colIndex][charIndex] = '\0'; // Null-terminate the string
      colIndex++;
      charIndex = 0; // Reset character index for the next column
      
      // Safety check: stop if max columns reached
      if (colIndex >= maxCols) break;

    }
	else if (c != '\r' && c != '\n') { // Ignore carriage return and newline characters
      // Append character to the current string
      if (charIndex < maxLen - 1) { // -1 for the null terminator
        arr[colIndex][charIndex] = c;
        charIndex++;
      } else {
        Serial.println("Warning: String truncated.");
      }
    }
  }
  
  // 3. Handle the Last Column
  // Null-terminate the last string after the loop finishes
  if (colIndex < maxCols && charIndex > 0) {
    arr[colIndex][charIndex] = '\0';
    colIndex++; // Increment to count the last column
  }

  return colIndex; // Return the number of columns successfully read
}

#endif