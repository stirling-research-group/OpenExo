

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "SendBulkChar.h"
/**
 * @brief Sends the contents of the txBuffer_bulkStr (the assembled CSV message) 
 * over the primary serial port and clears the buffer afterward.
 * * IMPORTANT: Serial.begin() must be called in setup() before this function runs.
 */
void send_bulk_char() {
    // 1. Check if the buffer is empty
    if (txBuffer_bulkStr[0] == '\0') {
        Serial.println("Warning: txBuffer_bulkStr is empty. Skipping UART send.");
        return;
    }
    
    // Calculate the exact length of the message.
    size_t message_length = strlen(txBuffer_bulkStr);
    
    // 2. Transmit the entire message in one burst using Serial.write().
    // This is the most efficient method for large C-strings on Arduino.
    Serial.write(txBuffer_bulkStr, message_length);
    
    // Optional: Add a small delay if the receiving end is slow to process data.
    // delay(5); 

    Serial.println("\n--- Message sent ---");

    // 3. Clear the buffer to prepare for the next assembly.
    // Crucial to prevent new, shorter messages from containing old data fragments.
    txBuffer_bulkStr[0] = '\0';
}
#endif