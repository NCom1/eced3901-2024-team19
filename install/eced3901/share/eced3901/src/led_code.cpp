//Put in the code for the lights below.

const int switchPin = 2;  // Pin connected to the Common terminal of the SPDT switch
const int ledPin = 3;    // Single pin connected to all LEDs through one resistor
const int blueLedPin = 4;  // Blue indicator LED
const int greenLedPin = 5; // Green indicator LED

int switchState = 0;  // Variable to store the current state of the switch
int lastSwitchState = 0; // Variable to store the previous state of the switch

bool mode = false; // Variable to toggle between 40Hz and 80Hz

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  pinMode(switchPin, INPUT);  // Set the switch pin as input
  lastSwitchState = digitalRead(switchPin);  // Initialize the last switch state

  pinMode(ledPin, OUTPUT); // Set the LED pin as output
  pinMode(blueLedPin, OUTPUT); // Set the blue LED pin as output
  pinMode(greenLedPin, OUTPUT); // Set the green LED pin as output
}

void loop() {
  switchState = digitalRead(switchPin);  // Read the current state of the switch

  if (switchState != lastSwitchState) {  // Check if the switch state has changed
    if (switchState == HIGH) {
      mode = true; // Set mode to 80Hz
      Serial.println("LEDs are emitting at 80Hz");
      digitalWrite(blueLedPin, LOW); // Turn off blue LED
      digitalWrite(greenLedPin, HIGH); // Turn on green LED
    } else {
      mode = false; // Set mode to 40Hz
      Serial.println("LEDs are emitting at 40Hz");
      digitalWrite(blueLedPin, HIGH); // Turn on blue LED
      digitalWrite(greenLedPin, LOW); // Turn off green LED
    }
    lastSwitchState = switchState;  // Update the last switch state
  }

  // Emit the appropriate frequency based on the mode
  if (mode) {
    // Emit 80Hz signal
    tone(ledPin, 80); // Emit 80Hz on ledPin
  } else {
    // Emit 40Hz signal
    tone(ledPin, 40); // Emit 40Hz on ledPin
  }
}


