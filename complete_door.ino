#include <DFRobot_ID809.h>

// Define fingerprint constants
#define COLLECT_NUMBER 3  // Fingerprint sampling times, can be set to 1-3
#define IRQ         6     // IRQ pin for fingerprint sensor
#define solenoidPin 2     // Pin for controlling the solenoid (door lock)

// Define TX and RX pins for hardware UART1 (fingerprint sensor)
#define SENSOR_TX  0      // GPIO for TX (Pico Pin 1, adjust based on your wiring)
#define SENSOR_RX  1      // GPIO for RX (Pico Pin 2, adjust based on your wiring)

DFRobot_ID809 fingerprint;

bool enrollMode = false;   // Flag to track enrollment mode
bool deleteMode = false;   // Flag to track deletion mode

void setup() {
  /* Init serial for communication */
  Serial.begin(9600);

  /* Init FPSerial (Serial1) */
  Serial1.begin(115200);  // Use hardware UART1 with 115200 baud rate
  fingerprint.begin(Serial1);

  /* Init solenoid pin */
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);  // Start with the solenoid locked (off)

  /* Test whether the fingerprint device can communicate */
  while (fingerprint.isConnected() == false) {
    Serial.println("Communication with fingerprint device failed, please check connection");
    delay(1000);
  }

  Serial.println("System Ready. Default mode: Fingerprint Comparison.");
  Serial.println("Commands:");
  Serial.println("'enroll' - Enter fingerprint enrollment mode");
  Serial.println("'delete' - Enter fingerprint deletion mode");
  Serial.println("'compare' - Switch to fingerprint comparison mode");
}

void loop() {
  // Check for serial commands to switch modes
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any extra spaces or newlines

    if (command.equals("enroll")) {
      Serial.println("Entering enrollment mode...");
      enrollMode = true;   // Set flag for enrollment mode
      deleteMode = false;  // Ensure deletion mode is off
    } 
    else if (command.equals("delete")) {
      Serial.println("Entering deletion mode...");
      deleteMode = true;   // Set flag for deletion mode
      enrollMode = false;  // Ensure enrollment mode is off
    }
    else if (command.equals("compare")) {
      Serial.println("Switching to comparison mode...");
      enrollMode = false;
      deleteMode = false;
    } else {
      Serial.println("Invalid command.");
    }
  }

  // Check for fingerprint input via IRQ
  if (digitalRead(IRQ)) {
    if (enrollMode) {
      fingerprintRegistration();  // Enroll the fingerprint when in enroll mode
    } else if (deleteMode) {
      fingerprintDeletion();      // Delete fingerprint when in delete mode
    } else {
      fingerprintMatching();      // Default behavior: compare fingerprints
    }
  }
}

// Compare fingerprints (default mode)
void fingerprintMatching() {
  Serial.println("Collecting fingerprint for comparison...");
  
  // Collect fingerprint image before comparing
  if (fingerprint.collectionFingerprint(5) != ERR_ID809) {
    Serial.println("Fingerprint captured, now searching for a match...");
    
    // Search for matching fingerprint in the library
    uint8_t ret = fingerprint.search();
    
    if (ret != 0) {
      fingerprint.ctrlLED(fingerprint.eKeepsOn, fingerprint.eLEDGreen, 0);
      Serial.print("Successfully matched, ID = ");
      Serial.println(ret);
      
      // Unlock the door by activating the solenoid
      digitalWrite(solenoidPin, HIGH);  // Activate solenoid (unlock)
      Serial.println("Door unlocked.");
      
      // Keep the door unlocked for 5 seconds, then lock again
      delay(5000);
      digitalWrite(solenoidPin, LOW);  // Deactivate solenoid (lock)
      Serial.println("Door locked again.");
    } else {
      fingerprint.ctrlLED(fingerprint.eKeepsOn, fingerprint.eLEDRed, 0);
      Serial.println("Matching failed, door remains locked.");
    }
  } else {
    Serial.println("Fingerprint capture failed.");
  }

  // Turn off LED after delay
  delay(1000);
  fingerprint.ctrlLED(fingerprint.eNormalClose, fingerprint.eLEDBlue, 0);
}

// Enroll fingerprints (switched via serial command)
void fingerprintRegistration() {
  uint8_t ID, i = 0;

  if ((ID = fingerprint.getEmptyID()) == ERR_ID809) {
    Serial.println("No empty ID available for enrollment.");
    return;
  }

  Serial.print("Unregistered ID, ID = ");
  Serial.println(ID);

  while (i < COLLECT_NUMBER) {
    fingerprint.ctrlLED(fingerprint.eBreathing, fingerprint.eLEDBlue, 0);  // Blue LED for capturing
    Serial.print("Fingerprint sampling (");
    Serial.print(i + 1);
    Serial.println(" of 3)");

    if (fingerprint.collectionFingerprint(10) != ERR_ID809) {
      fingerprint.ctrlLED(fingerprint.eFastBlink, fingerprint.eLEDYellow, 3);  // Yellow LED for success
      Serial.println("Fingerprint captured successfully.");
      i++;
    } else {
      Serial.println("Fingerprint capture failed.");
    }

    while (fingerprint.detectFinger());  // Wait until the finger is removed
  }

  if (fingerprint.storeFingerprint(ID) != ERR_ID809) {
    Serial.print("Fingerprint saved successfully, ID = ");
    Serial.println(ID);
    fingerprint.ctrlLED(fingerprint.eKeepsOn, fingerprint.eLEDGreen, 0);   // Green LED for success
  } else {
    Serial.println("Fingerprint saving failed.");
    fingerprint.ctrlLED(fingerprint.eKeepsOn, fingerprint.eLEDRed, 0);     // Red LED for failure
  }
  delay(1000);
  fingerprint.ctrlLED(fingerprint.eNormalClose, fingerprint.eLEDBlue, 0);  // Turn off LED
}

// Delete fingerprints (switched via serial command)
void fingerprintDeletion() {
  uint8_t ret = fingerprint.search();
  if (ret) {
    fingerprint.ctrlLED(fingerprint.eKeepsOn, fingerprint.eLEDGreen, 0);  // Green LED for success
    fingerprint.delFingerprint(ret);
    Serial.print("Deleted fingerprint, ID = ");
    Serial.println(ret);
  } else {
    fingerprint.ctrlLED(fingerprint.eKeepsOn, fingerprint.eLEDRed, 0);    // Red LED for failure
    Serial.println("Matching failed or fingerprint not registered.");
  }
  delay(1000);
  fingerprint.ctrlLED(fingerprint.eNormalClose, fingerprint.eLEDBlue, 0); // Turn off LED
}
