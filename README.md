# Zheng-Huang
EMK Project
/*
 * PI Stromregelung (PI Current Control)
 * * Steuert den Strom durch den L298-Treiber basierend auf der Messung
 * von einem 1.5 Ohm Messwiderstand.
 */

// --- 1. L298 Ansteuerungspins ---
const int ENA_PIN = 9;  // PWM Pin (muss ein '~' Pin sein)
const int IN1_PIN = 8;
const int IN2_PIN = 7;

// --- 2. Feedback (Sensor) Pin ---
const int CURRENT_SENSOR_PIN = A0; // Pin für den Spannungsabfall am Widerstand

// --- 3. PI-Regler Parameter (Müssen experimentell eingestellt werden!) ---
double Kp = 100.0; // Proportional-Anteil (Startwert)
double Ki = 20.0;  // Integral-Anteil (Startwert)

// --- 4. Widerstandswert (aus Ihrer Präsentation) ---
const double MESSWIDERSTAND_OHM = 1.5; // 1.5 Ohm 

// --- 5. Zielwert (Sollwert) ---
double setpointCurrent_Amps = 0.5; // Ziel: 0.5 Ampere

// --- Globale Variablen für den Regler ---
double integralSum = 0;   // Summe für den I-Anteil
double pwmOutput = 0;     // PWM-Wert (0-255)

// --- Zeitsteuerung für den Regler ---
unsigned long lastTime = 0;
long sampleTime_ms = 10; // Regler alle 10ms ausführen

void setup() {
  Serial.begin(115200);

  // L298 Pins als Ausgang setzen
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Setzt die Richtung des Stroms.
  // Der PI-Regler steuert nur die Stärke (ENA).
  // z.B. Verstärkend:
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
}

void loop() {
  unsigned long now = millis();
  
  // Führt den Regler nur alle 'sampleTime_ms' Millisekunden aus
  if (now - lastTime >= sampleTime_ms) {
    lastTime = now;

    // --- SCHRITT A: IST-STROM MESSEN (Input) ---
    
    // 1. Rohwert vom ADC lesen (0-1023)
    int analogValue = analogRead(CURRENT_SENSOR_PIN);

    // 2. Rohwert in Spannung umrechnen (0.0 - 5.0V)
    // Das Arduino Uno R4 Minima hat standardmäßig 10 Bit Auflösung (1023)
    double measuredVoltage = analogValue * (5.0 / 1023.0);

    // 3. Spannung in Strom umrechnen (I = U / R)
    double measuredCurrent_Amps = measuredVoltage / MESSWIDERSTAND_OHM;
    
    // --- SCHRITT B: PI-ALGORITHMUS ANWENDEN ---

    // 1. Fehler berechnen (Regelabweichung)
    double error = setpointCurrent_Amps - measuredCurrent_Amps;

    // 2. Integral-Anteil berechnen (Summen-Form)
    integralSum += (Ki * error);

    // 3. Integral-Begrenzung (Anti-Windup)
    // Verhindert, dass der I-Anteil unendlich groß wird.
    // Wir begrenzen ihn auf den PWM-Bereich (0-255).
    if (integralSum > 255.0) integralSum = 255.0;
    if (integralSum < 0.0) integralSum = 0.0;

    // 4. Gesamt-Ausgang berechnen (P-Anteil + I-Anteil)
    pwmOutput = (Kp * error) + integralSum;

    // --- SCHRITT C: AUSGANG (PWM) SETZEN ---
    
    // 1. Ausgang auf den gültigen PWM-Bereich (0-255) begrenzen
    if (pwmOutput > 255.0) pwmOutput = 255.0;
    if (pwmOutput < 0.0) pwmOutput = 0.0;

    // 2. PWM-Signal an den L298 senden
    analogWrite(ENA_PIN, (int)pwmOutput);

    // (Optional) Debug-Daten an den Serial Monitor senden
    Serial.print("Setpoint: "); Serial.print(setpointCurrent_Amps);
    Serial.print(" A, Measured: "); Serial.print(measuredCurrent_Amps);
    Serial.print(" A, PWM: "); Serial.println((int)pwmOutput);
  }
}
