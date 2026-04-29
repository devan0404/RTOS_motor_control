/*
 * ATmega328P Server - G-Code Forwarder
 * 
 * Hardware: Arduino Uno / Arduino Nano (ATmega328P)
 * 
 * IMPORTANT: In Arduino IDE, select:
 *   - Tools → Board → Arduino Uno (or Arduino Nano)
 *   - Tools → Port → Your ATmega's COM port
 * 
 * Function:
 *   - Receives G-code commands from PuTTY via USB Serial (Pins 0/1)
 *   - Forwards commands to ESP32 via SoftwareSerial (Pins 10/11)
 *   - Receives acknowledgments from ESP32
 *   - Displays responses in PuTTY
 * 
 * Pin Configuration:
 *   Pin 0 (RX) - USB Serial from PuTTY (hardware)
 *   Pin 1 (TX) - USB Serial to PuTTY (hardware)
 *   Pin 10 (RX) - Receives from ESP32 TX (GPIO17)
 *   Pin 11 (TX) - Sends to ESP32 RX (GPIO16)
 * 
 * Baud Rates:
 *   USB Serial: 115200
 *   ESP32 Serial: 115200
 * 
 * Protocol:
 *   PuTTY → ATmega → ESP32
 *   ESP32 → ATmega → PuTTY (acknowledgments)
 * 
 * TROUBLESHOOTING:
 *   If you get "SoftwareSerial does not name a type" error:
 *   1. Verify board is set to "Arduino Uno" or "Arduino Nano"
 *   2. Install SoftwareSerial: Sketch → Include Library → SoftwareSerial
 *   3. Or manually install from: https://github.com/PaulStoffregen/SoftwareSerial
 */

// SoftwareSerial is built-in for Arduino Uno/Nano
// If you get errors, make sure board is selected correctly
#include 

// Software Serial for ESP32 communication
// Pin 12 = RX (receives from ESP32 TX - GPIO17)
// Pin 13 = TX (sends to ESP32 RX - GPIO16)
SoftwareSerial esp32Serial(12, 13);  // RX, TX

// Buffer for incoming commands from PuTTY
String usbBuffer = "";

// Buffer for responses from ESP32
String esp32Buffer = "";

void setup() {
  // Initialize USB Serial (PuTTY connection)
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n========================================");
  Serial.println("  ATmega328P G-Code Server Started");
  Serial.println("========================================");
  Serial.println("Role: Command Forwarder");
  Serial.println("Receiving from: PuTTY (USB)");
  Serial.println("Forwarding to: ESP32 (Pins 12/13)");
  Serial.println("Pin 12 = RX from ESP32");
  Serial.println("Pin 13 = TX to ESP32");
  Serial.println("Baud Rate: 115200");
  Serial.println("========================================\n");
  
  // Initialize Software Serial for ESP32
  esp32Serial.begin(115200);
  delay(500);
  
  Serial.println("[READY] Type G-code commands below:");
  Serial.println("Examples:");
  Serial.println("  G1 X50 Y30 F50");
  Serial.println("  M3");
  Serial.println("  M5");
  Serial.println("  DRAW SQUARE 40");
  Serial.println("  S (status)");
  Serial.println("----------------------------------------\n");
}

void loop() {
  // ========================================
  // Read from USB (PuTTY) and forward to ESP32
  // ========================================
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (usbBuffer.length() > 0) {
        // Echo what was typed
        Serial.print(">> ");
        Serial.println(usbBuffer);
        
        // Forward to ESP32
        esp32Serial.println(usbBuffer);
        
        Serial.println("[Sent to ESP32] Waiting for response...");
        
        // Clear buffer
        usbBuffer = "";
      }
    } else {
      usbBuffer += c;
    }
  }
  
  // ========================================
  // Read responses from ESP32 and display in PuTTY
  // ========================================
  while (esp32Serial.available()) {
    char c = esp32Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (esp32Buffer.length() > 0) {
        // Display ESP32 response
        Serial.print("[ESP32 Response] ");
        Serial.println(esp32Buffer);
        Serial.println();  // Extra newline for readability
        
        // Clear buffer
        esp32Buffer = "";
      }
    } else {
      esp32Buffer += c;
    }
  }
}

/*
 * DEBUGGING TIPS:
 * 
 * 1. If ESP32 doesn't respond:
 *    - Check wiring (Pin 11 → GPIO16, Pin 10 → GPIO17)
 *    - Check GND connection between devices
 *    - Verify baud rate matches (115200)
 * 
 * 2. If you see garbled text:
 *    - Reduce baud rate to 57600 or 9600
 *    - Check for loose connections
 * 
 * 3. Monitor both Serial ports:
 *    - Open Serial Monitor on ATmega's port
 *    - Open another Serial Monitor on ESP32's USB port
 *    - Compare what each device sees
 */
