#include <pwm.h>

// ================= Datenarray-Einstellungen (21 Datensätze) =================
float arraySollKraftLA[41] = {
   4.0,  4.0,  4.0,  4.0,  4.0,  4.0,  4.0,  4.0, 4.0, 4.0, 4.0,  4.0,  4.0,  4.0,  4.0,  4.0,  4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0,
   4.0,  4.0,  4.0,  4.0,  4.0,  4.0,  4.0,  4.0, 4.0, 4.0, 4.0
}; 

float arraySollStromMSM[41] = {
   -1.0,-0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0,
   -0.1,-0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9, -1.0
};

// ================= Hardware-Pins und Objekte =================
PwmOut AusgabeStromLA(5);
PwmOut AusgabeStromMSM(6);

const int analogStromLA = A4;  // LA Ist-Strom Einlesen
const int analogStromSP = A5;  // MSM Ist-Strom Einlesen 
const int inLaser = A1;        // Laser-Wegsensor

int inLA1=12; int inLA2=13;
int inMSM1=9; int inMSM2=10;

// ================= Variablen und PI-Parameter =================
float nullReferenzLaser = 0.0;
float auslenkung = 0.0;
float dehnungMSM = 0.0;
String befehl = "";

// LA PI-Parameter 
float KpLA = 0.1, KiLA = 2;
float integralLA = 0;

// MSM PI-Parameter 
float KpMSM = 30.0, KiMSM = 20.0;
float integralMSM = 0;

int laengeArrayLA = 0;
int laengeArrayMSM = 0;
float istStromLA = 0;
float gemessenKraftLA = 0;
float pwm_out_LA = 0;
float pwm_out_MSM =0;


void setup() {
  Serial.begin(9600); 

  AusgabeStromLA.begin(20000.0f, 0.0f);
  AusgabeStromMSM.begin(20000.0f, 0.0f);

  pinMode(analogStromLA, INPUT);
  pinMode(analogStromSP, INPUT); 
  pinMode(inLaser, INPUT);

  pinMode(inLA1, OUTPUT); pinMode(inLA2, OUTPUT);
  digitalWrite(inLA1, LOW); digitalWrite(inLA2, HIGH);
  pinMode(inMSM1, OUTPUT); pinMode(inMSM2, OUTPUT);
  digitalWrite(inMSM1, HIGH);digitalWrite(inMSM2, LOW);

  laengeArrayLA = sizeof(arraySollKraftLA) / sizeof(arraySollKraftLA[0]);
  laengeArrayMSM = sizeof(arraySollStromMSM) / sizeof(arraySollStromMSM[0]);

  if (laengeArrayLA != laengeArrayMSM){
    Serial.println("ERROR: Length Mismatch!");
    while(1);
  }
  
  delay(1000);
  Serial.println("READY"); 
}

void loop() {
  if (Serial.available()) {
    befehl = Serial.readStringUntil('\n');
    befehl.trim();
  }
  if (befehl == "Neu") {

    // Tabellenkopf drucken
    
    // 1. Initiale starke Anpress-Rücksetzung 
    // ----------------------------------------------------
    // A. MSM-Kristall erweichen (Zur Kompensation des Magnetfelds)
    digitalWrite(inMSM1, HIGH);
    digitalWrite(inMSM2, LOW);
    float sollStromMSM =-1.0;
    float sollPWMMSM = (-sollStromMSM)*48.03281581 + 51.78413903;
    AusgabeStromMSM.pulse_perc(sollPWMMSM); 

    // B. LA legt initiale Rücksetzkraft von 8,6N an (Open-Loop-Formel für sofortigen Kraftaufbau)
    float initKraftLA = 8.6;
    float initStromLA = initKraftLA / 6.0;
    float initPWMLA = initStromLA * 36.6571359 + 46.71291013;
    AusgabeStromLA.pulse_perc(initPWMLA);

    delay(1500); 
    // C. Laser-Nullpunkt im kürzesten, vollständig komprimierten Zustand einlesen
    // ---------------------------------------------------
    float rawZero = analogRead(inLaser);
    nullReferenzLaser = (rawZero * 5.0 / 1023.0) * 7.7 / 3.98;
    delay(1000); 

    Serial.println("START_DATA"); 
    Serial.println("SollLAKraft, SollStromMSM, Dehnung, pwm_out_LA, pwm_out_MSM"); 


    // 2. Messzyklus
    for (int i = 0; i < laengeArrayMSM; i++) {
      
      float sollKraftLA = arraySollKraftLA[i];
      float sollStromMSM = arraySollStromMSM[i];
      
      float zielStromLA = sollKraftLA / 6.0;
      zielStromLA = constrain(zielStromLA, 0.0, 1.5);

      
      // MSM Richtungssteuerung
      float zielStromMSM_abs = abs(sollStromMSM);
      
      if (sollStromMSM < 0) {
        digitalWrite(inMSM1, HIGH); digitalWrite(inMSM2, LOW);
      } else {
        digitalWrite(inMSM1, LOW); digitalWrite(inMSM2, HIGH);
      }

      // Integralanteil zurücksetzen, Vorbereitung auf neuen Sollwert
      integralLA = 0;
      integralMSM = 0;


      float basePWMLA = zielStromLA * 36.6571359 + 46.71291013;

      // ========================================================
      // 2,0 Sekunden PI-Regelkreis-Steuerung 
      // ========================================================
      long sampleTime_ms = 10;       
      unsigned long duration = 2000; 
      unsigned long startTime = millis();
      unsigned long lastSampleTime = millis();
      
      float gemessenerStromMSM = 0.0; 
      float finalIstStromLA = 0.0; // Speichert den letzten Ist-Stromwert
      
      while (millis() - startTime < duration) {
        unsigned long now = millis();
        
        if(now - lastSampleTime >= sampleTime_ms) {
          lastSampleTime = now;
          
          // --- A. MSM PI-Regelkreis ---
          float istStromSP = (analogRead(analogStromSP) * 5.0) / (1023.0 * 1.5);
          gemessenerStromMSM = istStromSP;

          float errorMSM = zielStromMSM_abs - istStromSP;
          integralMSM += (KiMSM * errorMSM);
          integralMSM = constrain(integralMSM, -100, 100); 
          pwm_out_MSM = (KpMSM * errorMSM) + integralMSM;
          pwm_out_MSM = constrain(pwm_out_MSM, 0.0, 100.0);
          AusgabeStromMSM.pulse_perc(pwm_out_MSM);

          // --- B. LA PI-Regelkreis ---
          istStromLA = (analogRead(analogStromLA) * 5.0) / (1023.0 * 1.5);
          //finalIstStromLA = istStromLA; // Letzten Stromwert aufzeichnen
          //float errorLA = zielStromLA - istStromLA;
          //integralLA += ( KiLA * errorLA );
          //integralLA = constrain(integralLA, -20, 20); 
          pwm_out_LA = basePWMLA;// + (KpLA * errorLA) + integralLA;
          pwm_out_LA = constrain(pwm_out_LA, 0.0, 100.0);
          AusgabeStromLA.pulse_perc(pwm_out_LA);

        }
      } 
      // === Ende der PI-Stabilisierungsphase ===

      // C. Nach der Stabilisierung die Verschiebung messen (ohne abs(), um Vorzeichen beizubehalten)
      float rawVal = analogRead(inLaser);
      auslenkung = (rawVal * 5.0 / 1023.0) * 7.7 / 3.98;
      dehnungMSM = ((nullReferenzLaser - auslenkung) / 15.0) * 100.0;
      gemessenKraftLA = istStromLA * 6;

      // Wahres Vorzeichen des MSM-Iststroms wiederherstellen
      float printStromMSM = (sollStromMSM < 0) ? -gemessenerStromMSM : gemessenerStromMSM;

      // D. Einzeilige Datenausgabe
      Serial.print(sollKraftLA); 
      Serial.print(", "); 
      Serial.print(sollStromMSM); 
      Serial.print(", "); 
      Serial.print(dehnungMSM); 
      Serial.print(", "); 
      Serial.print(pwm_out_LA);  
      Serial.print(", "); 
      Serial.println(pwm_out_MSM); 
    } 
    
    // Abschluss-Reset
    Serial.println("END_DATA");
    AusgabeStromLA.pulse_perc(0.0f);
    AusgabeStromMSM.pulse_perc(0.0f);

    befehl = "Done"; 
  }
}