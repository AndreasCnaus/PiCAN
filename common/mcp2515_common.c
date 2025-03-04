#include "mcp2515_common.h"

const char *mode_strings[] = {
    "Normal Operation mode",  // 0x00
    "Sleep mode",             // 0x01
    "Loopback mode",          // 0x02
    "Listen-Only mode",       // 0x03
    "Configuration mode"      // 0x04
};

size_t get_opmode_strings_len(void) {
    return sizeof(mode_strings) / sizeof(mode_strings[0]);
}

const char* get_opmode_string(unsigned int mode) {
    if (mode >= 0 && mode <= 4) {
        return mode_strings[mode];
    } else {
        return "Invalid operation mode";
    }
}