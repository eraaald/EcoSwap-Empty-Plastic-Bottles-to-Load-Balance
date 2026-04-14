#include <SoftwareSerial.h>

// Pins 2 and 3 for GSM
SoftwareSerial mySerial(2, 3); 

void setup() {
  Serial.begin(115200); 
  mySerial.begin(9600); 
  Serial.println("GSM Listener Ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();

    // Verify the command format
    if (incoming.startsWith("C:")) {
      // Dynamically extract the phone number sent by the Main Arduino
      String phoneNumber = incoming.substring(2); 
      
      // Validation: Ensure it looks like a phone number before sending
      if (phoneNumber.length() >= 10) {
        String message = "Receipt: You claimed your load. Enjoy!";
        
        Serial.print("Sending SMS to user: ");
        Serial.println(phoneNumber);
        
        sendSMS(phoneNumber, message);
      }
    }
  }

  // Relay module responses to your PC
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}

void sendSMS(String number, String message) {
  mySerial.println("AT+CMGF=1");
  delay(200);
  mySerial.print("AT+CMGS=\"");
  mySerial.print(number);
  mySerial.println("\"");
  delay(200);
  mySerial.print(message);
  delay(200);
  mySerial.write(26); // Ctrl+Z
  delay(2000);
  Serial.println(" >> Message sent to user!");
}