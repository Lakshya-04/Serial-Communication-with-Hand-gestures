char t;
void setup() {
  pinMode(2, OUTPUT);   // Left motor forward
  pinMode(3, OUTPUT);   // Left motor reverse
  pinMode(4, OUTPUT);   // Right motor forward
  pinMode(5, OUTPUT);
  Serial.begin(9600);  // Attaching the servo to the specified pin
}

void loop() {
  if (Serial.available()) {
   char t = Serial.read();
   if (t == 'F') {      // Move forward (both motors rotate in the forward direction)
    digitalWrite(2, HIGH);   // Left motor forward
    digitalWrite(3, LOW);    // Left motor reverse
    digitalWrite(4, HIGH);   // Right motor forward
    digitalWrite(5, LOW);    // Right motor reverse
    }
   else if (t == 'B') {      // Move backward (both motors rotate in the reverse direction) 
    digitalWrite(2, LOW);    // Left motor reverse
    digitalWrite(3, HIGH);   // Left motor forward
    digitalWrite(4, LOW);    // Right motor reverse
    digitalWrite(5, HIGH);   // Right motor forward
    }
   else if (t == 'L') {      // Turn left (left motor reverse, right motor forward)
    digitalWrite(2, LOW);    // Left motor reverse    
    digitalWrite(3, HIGH);   // Left motor forward
    digitalWrite(4, HIGH);   // Right motor forward
    digitalWrite(5, LOW);    // Right motor reverse
   }
   else if (t == 'R') {      // Turn right (left motor forward, right motor reverse)
    digitalWrite(2, HIGH);   // Left motor forward
    digitalWrite(3, LOW);    // Left motor reverse
    digitalWrite(4, LOW);    // Right motor reverse
    digitalWrite(5, HIGH);   // Right motor forward
   }
   else if (t == 'S'){      // Turn right (left motor forward, right motor reverse)
    digitalWrite(2, LOW);   // Left motor forward
    digitalWrite(3, LOW);    // Left motor reverse
    digitalWrite(4, LOW);    // Right motor reverse
    digitalWrite(5, LOW);   // Right motor forward
   }
  }
}
