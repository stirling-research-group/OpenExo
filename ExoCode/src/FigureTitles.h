#ifndef FIGURETITLES_H
#define FIGURETITLES_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "StatusDefs.h"
#include "Config.h"

//Specific Libraries
#include "ParseIni.h"
#include "ParamsFromSD.h"


#include <cstddef> // Required for size_t
#include <cstdint> // Required for uint8_t

// --- MOCKUP CONFIGURATION DEFINITIONS ---
/* namespace config_defs {    
    enum exo_name : uint8_t {
        bilateral_ankle = 1, // Exoskeleton Type 1
        default_mode = 99    // Default or unknown type
    };
} */
// ------------------------------------------

// Maximum buffer size estimate (must be large enough for "t," + 11 columns + 10 commas + ",z" + null terminator)
const size_t MAX_COMBINED_HEADER_LENGTH = 200; 

// --- Definition of the Mapping Function (INLINE, DYNAMIC) ---

/**
 * @brief Returns the column header string based on the configuration ID and 
 * the 0-based column index (0 to 10).
 */
inline const char* getColumnHeader(uint8_t column_index, uint8_t* config_to_send) {
    //using namespace config_defs;

    // Outer switch: Selects the set of headers based on the Exo Type ID
    switch (config_to_send[config_defs::exo_name_idx]) {
        
        case (uint8_t)config_defs::exo_name::bilateral_ankle:
        {
            // Inner switch: Selects the specific column name for this mode (0-based)
            switch (column_index) {
                case 0:  return "JointID";
                case 1:  return "ControllerID";
                case 2:  return "FileName";
                case 3:  return "Timestamp";
                case 4:  return "TargetValue";
                case 5:  return "CurrentValue";
                case 6:  return "Error";
                case 7:  return "P_Term";
                case 8:  return "I_Term";
                case 9:  return "D_Term";
                case 10: return "BattVol";
                default: return "INVALID_COL";
            }
        }
        
        // DEFAULT CASE: Generic Data Headers
        default:
        {
            switch (column_index) {
                case 0:  return "Data 0";
                case 1:  return "Data 1";
                case 2:  return "Data 2";
                case 3:  return "Data 3";
                case 4:  return "Data 4";
                case 5:  return "Data 5";
                case 6:  return "Data 6";
                case 7:  return "Data 7";
                case 8:  return "Data 8";
                case 9:  return "Data 9";
                case 10: return "Data 10";
                default: return "INVALID_COL";
            }
        }
    }
}

/**
 * @brief Function declaration to combine the column strings into a single delimited C-string.
 */
bool create_figure_titles(char* output_buffer, size_t buffer_size);

#endif
#endif




