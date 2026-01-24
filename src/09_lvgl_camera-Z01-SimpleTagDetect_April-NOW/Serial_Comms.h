//// jwc 26-0109-1730 Serial communications on GPIO 18 (TX) and GPIO 44 (RX) using UART1
//// jwc 26-0109-1730 Pin Selection Analysis:
//// * Available pins from [2,4, 6-21, 43-44, 47-48]:
////   - GPIO 2,4,6-17,21: Camera (cannot use)
////   - GPIO 47,48: Touch I2C (cannot use)
////   - GPIO 19,20: USB D-/D+ hardware (cannot use)
////   - GPIO 5: Not broken out on Waveshare board (cannot use)
////   - GPIO 43: UART0 TX (boot messages - would flood micro:bit buffer)
////   - GPIO 44: UART0 RX (safe - no outgoing boot data)
////   - GPIO 18: Free (currently used for RX, now repurposed for TX)
//// * Final choice: GPIO 44 (RX from micro:bit) + GPIO 18 (TX to micro:bit)
//// * Trade-off: Remapping GPIO 44 to UART1 disables UART0 RX (Serial Monitor input)
////   but Serial.print() debugging still works (uses GPIO 43 TX only)
#pragma once
#include "Arduino.h"

//// jwc 26-0109-1730 ARCHIVED: Previous RX-only configuration
//// #define SERIAL_COMMS_TX 17  // Disabled for camera PWDN
//// #define SERIAL_COMMS_RX 18

//// jwc 26-0109-1730 NEW: Bidirectional UART1 on GPIO 44 (RX) / GPIO 18 (TX)
#define SERIAL_COMMS_TX 18  // E3 TX → Micro:bit RX
#define SERIAL_COMMS_RX 44  // E3 RX ← Micro:bit TX
#define SERIAL_COMMS_BAUD 115200

//// jwc 26-0109-1520 NEW: Comm display buffer for scrolling text overlay
//// jwc 26-0110-0820 ARCHIVED: #define MAX_COMM_LINES 10
//// jwc 26-0110-0820 NEW: Increase to 20 lines for upper 75% of screen
#define MAX_COMM_LINES 20
extern String comm_lines[MAX_COMM_LINES];
extern int comm_line_count;
extern bool comm_display_enabled;

void Serial_Comms_Init();
void Serial_Comms_Send(const char* message);
void Serial_Comms_Task(void *parameter);
//// jwc 26-0109-1520 NEW: Add line to comm buffer (newest at top, push down)
void Comm_Add_Line(const char* line);
