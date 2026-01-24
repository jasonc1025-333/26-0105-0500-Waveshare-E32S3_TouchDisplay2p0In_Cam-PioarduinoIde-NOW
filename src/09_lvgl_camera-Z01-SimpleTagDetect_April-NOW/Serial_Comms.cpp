#include "Serial_Comms.h"

//// jwc 26-0109-1200 Create UART1 instance for external device communication
HardwareSerial SerialComms(1);

static char rx_buffer[256];
static int rx_index = 0;

//// jwc 26-0109-1520 NEW: Comm display buffer and state
String comm_lines[MAX_COMM_LINES];
int comm_line_count = 0;
bool comm_display_enabled = false;

void Serial_Comms_Init() {
  //// jwc 26-0109-1730 ARCHIVED: RX-only mode (TX disabled for camera PWDN testing)
  //// SerialComms.begin(SERIAL_COMMS_BAUD, SERIAL_8N1, SERIAL_COMMS_RX, -1);  // TX disabled
  //// printf("Serial Comms initialized: >>> E3_RX_GPIO18: (<<< E3_TX_GPIO17: Disabled for Camera Use) at %d baud\r\n", SERIAL_COMMS_BAUD);
  
  //// jwc 26-0109-1730 NEW: Bidirectional UART1 on GPIO 44 (RX) / GPIO 18 (TX)
  SerialComms.begin(SERIAL_COMMS_BAUD, SERIAL_8N1, SERIAL_COMMS_RX, SERIAL_COMMS_TX);
  //// jwc 26-0109-1940 ARCHIVED: printf("*** Serial Comms initialized: E3_RX_GPIO44, E3_TX_GPIO18 at %d baud\r\n", SERIAL_COMMS_BAUD);
  printf("*** Serial Comms initialized: E3_RX_GPIO44, E3_TX_GPIO18 at %d baud\n", SERIAL_COMMS_BAUD);
}

void Serial_Comms_Send(const char* message) {
  //// jwc 26-0109-1730 ARCHIVED: TX disabled for camera PWDN testing
  //// printf("<<< E3_TX_GPIO17: Disabled for Camera Use: %s\r\n", message);
  
  //// jwc 26-0109-1730 NEW: TX enabled on GPIO 18
  SerialComms.println(message);
  //// jwc 26-0109-1940 ARCHIVED: printf("*** E3>%s\r\n", message);
  printf("*** E3>%s\n", message);
}

void Serial_Comms_Task(void *parameter) {
  //// jwc 26-0109-1200 Track time for 3-second periodic TX
  uint32_t last_tx_time = 0;
  
  while(1) {
    //// jwc 26-0109-1200 Send "E3>%d:%d" every 3 seconds with random numbers
    uint32_t current_time = millis();
    if(current_time - last_tx_time >= 3000) {
      int rand1 = random(0, 5);  // Random number 0-4
      int rand2 = random(5, 10); // Random number 5-9
      char tx_msg[32];
      snprintf(tx_msg, sizeof(tx_msg), "E3>%d:%d", rand1, rand2);
      Serial_Comms_Send(tx_msg);
      last_tx_time = current_time;
    }
    
    //// jwc 26-0109-1730 Check if data is available on GPIO 44 RX
    //// jwc 26-0109-1730 Line-buffered: accumulates characters until newline
    while(SerialComms.available()) {
      char c = SerialComms.read();
      
      //// jwc 26-0109-1730 Process complete lines (not char-by-char)
      if(c == '\n' || c == '\r') {
        if(rx_index > 0) {
          rx_buffer[rx_index] = '\0';
          
          //// jwc 26-0109-1730 Print complete line to USB serial monitor
          //// jwc 26-0109-1940 ARCHIVED: printf("*** E3<%s\r\n", rx_buffer);
          //// jwc 26-0109-2000 ? extra blank line: printf("*** E3<%s\n", rx_buffer);
          //// jwc n printf("*** E3<%s", rx_buffer);
          printf("*** E3<%s\n", rx_buffer);
          
          //// jwc 26-0109-1520 NEW: Add received line to display buffer
          Comm_Add_Line(rx_buffer);
          
          rx_index = 0;
        }
      } else if(rx_index < sizeof(rx_buffer) - 1) {
        rx_buffer[rx_index++] = c;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//// jwc 26-0109-1520 NEW: Add line to comm buffer (newest at top, push older down)
void Comm_Add_Line(const char* line) {
  // Shift all existing lines down by one position
  for(int i = MAX_COMM_LINES - 1; i > 0; i--) {
    comm_lines[i] = comm_lines[i - 1];
  }
  
  // Insert new line at top (index 0)
  comm_lines[0] = String(line);
  
  // Track count (max MAX_COMM_LINES)
  if(comm_line_count < MAX_COMM_LINES) {
    comm_line_count++;
  }
}
