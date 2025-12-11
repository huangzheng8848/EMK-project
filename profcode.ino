//Notwendig, um die Frequenz des PWM-Signals verstellen zu können
#include <pwm.h>

//Setzen der Ausgänge für Tauchspulenaktor (Strom) und Spulenstrom
PwmOut AusgabeStromMSM(6);
PwmOut AusgabeStromLA(5);

//Initialisieren der Kanäle für das Setzen der Stromflussrichtung
//Motortreiber 1 - Spulen
int in1 = 9; 
int in2 = 10;

//Motortreiber 2 - Tauchspulenaktor
int in3 = 12; 
int in4 = 13;

//Eingänge der Messgrößen
const int inLaser = A1;
const int analogStromLA = A4;  
const int analogStromSP = A5;

//Stromregelung LA - hier nicht relevant, Viariablen schon initialisiert
float sollStromLA = 0;
float sollPWM_LA = 0;
float istStromLA = 0; 
float KpLA = 1, KiLA = 0.5, KdLA = 1;
float vorherigerFehlerLA = 0;
float integralLA = 0;
float derivativeLA = 0;
float lastTime = 0;
float fehlerAbsolutLA = 0;
float sollPWMLA = 0;

//Stromregelung MSM - hier nicht relevant, Viariablen schon initialisiert
float sollPWMMSM = 0;
float istStromSP = 0;
float KpMSM = 30, KiMSM = 2, KdMSM = 1;
float vorherigerFehlerMSM = 0;
float integralMSM = 0;

//Dehnungsregelung MSM - hier nicht relevant, Viariablen schon initialisiert
float sollDehnung = 0;
float sollStromMSM =0;
float KpDehnung = 0.01, KiDehnung = 0.01, KdDehnung = 0.01;
float vorherigerFehlerDehnung = 0;

//Messgrößen
float istDehnung = 0;
float istStromMSM = 0;
float istLaserSignal = 0;
float nullReferenzLaser = 0;
float current_pwm = 0;

//Deklaration für Eingabe im Loop:
enum Zustand {Normal, Wartet_W1, Wartet_W2, Wartet_W3};
Zustand status = Normal;

//Konstante um erneuten Programmdurchlauf zu verhindern/Reset
int i = 0;

//Initialisieren der Sollkraft
float sollKraftLA = 0;

//Ausgabegröße der Längenmessung
float auslenkung = 0;
float dehnungMSM = 0;


void setup() {

  Serial.begin(9600); // Serielle Kommunikation starten, erstmal Standardwert mit 9600 Bits

  //Initialisieren von Start-Werten und Ändern der Ausgabefrequenz auf 20 kHz:
  AusgabeStromLA.begin(20000.0f, 0.0f);
  AusgabeStromMSM.begin(20000.0f, 0.0f);

  //Setzen der "Richtung" des Spulenstroms für:
  //Motortreiber 1 (Spulenstrom): 
  pinMode(in1,OUTPUT); 
  pinMode(in2,OUTPUT);
  digitalWrite(in1,HIGH); 
  digitalWrite(in2,LOW);

  //Motortreiber 2 (Tauchspulenaktor):
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  digitalWrite(in3,LOW); 
  digitalWrite(in4,HIGH);

}

void loop() {
  String befehl = Serial.readStringUntil('\n');
  befehl.trim(); // Entfernt führende und nachfolgende Leerzeichen und Zeilenumbrüche

  //Starten der Eingabe über den Serial Monitor
  if (status == Normal && befehl == "Neu") {
        
        //Um einmaligen Durchlauf des Programmes zu gewährleisten
        i = 0;
        sollStromMSM=0.5;
        sollStromLA=0.5;
      }

  if (status == Normal && i == 0){
    Serial.print("start");

  
    ////////////////////////////////////////////////////////////////////////////
    // PI-Regeler Laufzeit 
    long sampleTime_ms = 10;
    unsigned long duration = 2000;
    unsigned long startTime = millis();
    unsigned long lastSampleTime = millis();
    
    while (millis() - startTime < 2000) {
      unsigned long now = millis();
      if(now - lastSampleTime >= sampleTime_ms){
      lastSampleTime = now;
      
      istStromSP = (analogRead(analogStromSP) * 5.0) / (1023.0 * 1.5);
  
      // Positional PI
      float error = sollStromMSM - istStromSP;
      integralMSM += (KiMSM * error);
      integralMSM = constrain(integralMSM, -80, 80); // Anti-Windup
      float pwm_out = (KpMSM * error) + integralMSM;
      pwm_out = constrain(pwm_out, 0.0, 100.0);
      
      AusgabeStromMSM.pulse_perc(pwm_out);
      current_pwm = pwm_out;
///////////////////////////////////////// stromlast part
      istStromLA = (analogRead(analogStromLA) * 5.0) / (1023.0 * 1.5);
        
      float errorLA = sollStromLA - istStromLA;
      integralLA += (KiLA * errorLA);
      integralLA = constrain(integralLA, -70, 70); // Anti-Windup
        
        // Berechnung des neuen PWM Werts
        // Man kann den vorher berechneten "sollPWMLA" als Basis (Feedforward) nehmen und den Regler nur die Differenz machen lassen
        // Oder man lässt den Regler alles machen. Hier einfacher PI-Ansatz:
      float pwm_out_LA = (KpLA * errorLA) + integralLA; 
        
        // Falls du den Startwert (Feedforward) nutzen willst, nutze stattdessen:
        // float pwm_out_LA = sollPWMLA + (KpLA * errorLA) + integralLA;
        
      pwm_out_LA = constrain(pwm_out_LA, 0.0, 100.0);
      AusgabeStromLA.pulse_perc(pwm_out_LA);

    Serial.print("gemessener Spulenstrom ist "); Serial.println(istStromSP);
    Serial.print("gemessener Lastaktorstrom ist "); Serial.println(istStromLA);
 ////////////////////////////////////////////////////       
      }
    }

    
    
  
    //Erneute Messung der Auslenkung und Berechnung der Dehnung
    auslenkung = (analogRead(inLaser)*5/1023.0)*7.7/3.98; // 7,7 mm pro 3,98 V
    dehnungMSM =  abs(((auslenkung - nullReferenzLaser)/15)*100);

    Serial.println("Kraft, Spulestrom, Dehnung");
    Serial.print(sollKraftLA);
    Serial.print(", ");
    Serial.print(sollStromMSM);
    Serial.print(", ");
    Serial.print("Dehnung in [%]: "); Serial.println(dehnungMSM);
    i = 1;
  }  

  //Zurücksetzen der Werte auf 0, um Überhitzung zu vermeiden 
  if (status == Normal && i == 1){
    AusgabeStromMSM.pulse_perc(0.0f);
    AusgabeStromLA.pulse_perc(0.0f);
  }
}