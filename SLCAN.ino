/*
 ___ ___   _____ __    _  _ __  _ _____ _ _     __ ___  __  _  _____ __  _  
| __| \ \_/ | __| _\  | || |  \| |_   _| | |   |  | _ \/__\| |/ | __|  \| | 
| _|| |> , <| _|| v | | \/ | | ' | | | | | |_  | -| v | \/ |   <| _|| | ' | 
|_| |_/_/ \_|___|__/   \__/|_|\__| |_| |_|___| |__|_|_\\__/|_|\_|___|_|\__|
Garner M 
2025
Arduino-based SLCAN. This build is for SWCAN GMLAN.
V0.1 Work in progress
V0.2 Send Frame Function now works
*/
#include <SPI.h>
#include <mcp2515.h>

#define CMD_LEN (sizeof("T12345678811223344556677881234\r")+1)

const int MODE0_PIN = 3;
const int MODE1_PIN = 2;
struct can_frame canMsg;
MCP2515 mcp2515(10); // CS on pin 10

void setup() {
  Serial.begin(1000000);
  
  // Set the TH8056 to 33.3 Kbps mode
  pinMode(MODE0_PIN, OUTPUT);
  pinMode(MODE1_PIN, OUTPUT);
  digitalWrite(MODE0_PIN, HIGH);
  digitalWrite(MODE1_PIN, HIGH);
  
  // Initialize CAN controller
  mcp2515.reset();
  mcp2515.setBitrate(CAN_33KBPS); // GM LAN low-speed mode
  mcp2515.setNormalMode();
}

void loop() {
  Serial2Can(); // Handle incoming serial commands
  Can2Serial(); // Forward received CAN messages to serial
}

// Helper functions
void slcan_ack() { Serial.write('\r'); }
void slcan_nack() { Serial.write('\a'); }

// Transfer from CAN to Serial (SLCAN format)
void Can2Serial() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    char buf[CMD_LEN];
    char *p = buf;
    
    // Detect extended frame correctly
    bool isExtended = (canMsg.can_id & CAN_EFF_FLAG) || (canMsg.can_id > 0x7FF);

    if (isExtended) {
      *p++ = 'T'; // Extended frame
      uint32_t id = canMsg.can_id & CAN_EFF_MASK; // Only 29 bits
      p += sprintf(p, "%08lX", id); // Exactly 8 hex digits
    } else {
      *p++ = 't'; // Standard frame
      uint32_t id = canMsg.can_id & CAN_SFF_MASK; // Only 11 bits
      p += sprintf(p, "%03X", id); // Exactly 3 hex digits
    }
    
    // Add DLC (data length code)
    p += sprintf(p, "%1X", canMsg.can_dlc & 0xF); // 0-8

    // Add data bytes if not RTR
    if (!(canMsg.can_id & CAN_RTR_FLAG)) {
      for (uint8_t i = 0; i < canMsg.can_dlc; i++) {
        p += sprintf(p, "%02X", canMsg.data[i]);
      }
    }
    
    *p++ = 0x0D; // SLCAN terminator
    *p = '\0';   // Null-terminate string

    Serial.print(buf);
  }
}

// Transfer from Serial to CAN (parse SLCAN commands)
void Serial2Can() {
  static char cmdbuf[CMD_LEN];
  static int cmdidx = 0;

  while (Serial.available()) {
    char val = Serial.read();
    cmdbuf[cmdidx++] = val;

    if (cmdidx >= CMD_LEN) { // command too long
      slcan_nack();
      cmdidx = 0;
    } else if (val == '\r') { // end of command
      cmdbuf[cmdidx] = '\0';
      pars_slcancmd(cmdbuf);
      cmdidx = 0;
    }
  }
}

// Parse SLCAN commands
void pars_slcancmd(char *buf) {
  switch (buf[0]) {
    case 'O': // Open channel
      mcp2515.reset();
      mcp2515.setBitrate(CAN_33KBPS);
      mcp2515.setNormalMode();
      slcan_ack();
      break;
      
    case 'C': // Close channel
      mcp2515.reset();
      slcan_ack();
      break;
      
    case 't': // Standard frame
    case 'T': // Extended frame
    case 'r': // RTR Standard frame
    case 'R': // RTR Extended frame
      send_canmsg(buf);
      break;
      
    case 'S': // Set bitrate
      set_bitrate(buf[1]);
      break;
      
    case 'Z': // Timestamp
      slcan_ack(); // Just acknowledge, we're not implementing timestamps
      break;
      
    case 'F': // Status flag
      Serial.print("F12"); // Basic status
      slcan_ack();
      break;
      
    case 'V': // Version
      Serial.print("GM_SLCAN_1.0");
      slcan_ack();
      break;
      
    case 'N': // Serial number
      Serial.print("12345");
      slcan_ack();
      break;
      
    default:
      slcan_nack();
      break;
  }
}

void send_canmsg(char *buf) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  
  // Determine frame type
  bool isExtended = (buf[0] == 'T' || buf[0] == 'R');
  bool isRTR = (buf[0] == 'r' || buf[0] == 'R');
  
  // Parse ID
  char idStr[9] = {0};
  int dlc_pos;
  
  if (isExtended) {
    strncpy(idStr, buf+1, 8); // Get 8-character ID
    frame.can_id = strtoul(idStr, NULL, 16) | CAN_EFF_FLAG;
    dlc_pos = 9; // DLC position for extended frames
  } else {
    strncpy(idStr, buf+1, 3); // Get 3-character ID
    frame.can_id = strtoul(idStr, NULL, 16);
    dlc_pos = 4; // DLC position for standard frames
  }
  
  // Parse DLC (data length code)
  if (strlen(buf) > dlc_pos) {
    frame.can_dlc = buf[dlc_pos] - '0';
    if (frame.can_dlc > 8) frame.can_dlc = 8;
  } else {
    frame.can_dlc = 0;
  }
  
  // Set RTR flag if needed
  if (isRTR) {
    frame.can_id |= CAN_RTR_FLAG;
  }
  // Parse data bytes (if not RTR and data exists)
  else if (strlen(buf) > dlc_pos + 1) {
    int dataStart = dlc_pos + 1;
    int dataLen = strlen(buf) - dataStart;
    
    for (int i = 0; i < frame.can_dlc && (i*2) < dataLen; i++) {
      char byteStr[3] = {0};
      strncpy(byteStr, buf + dataStart + (i*2), 2);
      frame.data[i] = strtoul(byteStr, NULL, 16);
    }
  }
  
  // Send the frame
  if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK) {
    slcan_ack();
  } else {
    slcan_nack();
  }
}

// Set CAN bitrate
void set_bitrate(char rate) {
  switch (rate) {
    case '0': // 10k (not supported)
    case '1': // 20k (not supported)
    case '2': // 50k (not supported)
      slcan_nack();
      break;
      
    case '3': // 100k
      mcp2515.setBitrate(CAN_100KBPS);
      slcan_ack();
      break;
      
    case '4': // 125k
      mcp2515.setBitrate(CAN_125KBPS);
      slcan_ack();
      break;
      
    case '5': // 250k
      mcp2515.setBitrate(CAN_250KBPS);
      slcan_ack();
      break;
      
    case '6': // 500k (GM LAN high-speed)
      mcp2515.setBitrate(CAN_500KBPS);
      slcan_ack();
      break;
      
    case 'G': // Custom: GM LAN 33.3k (not standard SLCAN)
      mcp2515.setBitrate(CAN_33KBPS);
      slcan_ack();
      break;
      
    default:
      slcan_nack();
      break;
  }
}
