//Notwendig, um die Frequenz des PWM-Signals verstellen zu können
#include <pwm.h>; 

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
float KpLA = 0.001, KiLA = 0.001, KdLA = 0.001;
float vorherigerFehlerLA = 0;
float integralLA = 0;
float derivativeLA = 0;
float lastTime = 0;
float fehlerAbsolutLA = 0;
float sollPWMLA = 0;

//Stromregelung MSM - hier nicht relevant, Viariablen schon initialisiert
float sollPWMMSM = 0;
float istStromSP = 0;
float KpMSM = 0.01, KiMSM = 0.01, KdMSM = 0.01;
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

  //Starten der Eingabe über den Serial Monitor
  if (Serial.available()) {
      String befehl = Serial.readStringUntil('\n');
      befehl.trim(); // Entfernt führende und nachfolgende Leerzeichen und Zeilenumbrüche

      //Starten des Eingabebefehls durch "Neu"
      if (status == Normal && befehl == "Neu") {
        Serial.println("Konfigurationsmodus aktiv!");
        Serial.println("Geben Sie einen neuen Wert für die Kraft des Last-Aktors ein (0 - 8 N).");
        status = Wartet_W1;
      } else if (status == Wartet_W1) {
        sollKraftLA = befehl.toFloat();
        Serial.println("Geben Sie einen neuen Wert für die Stromstärke der Spulen ein(-1.0 (Verstärkend) bis 1.0 (kompensierend)).");
        status = Wartet_W2;
      } else if (status == Wartet_W2) {
        sollStromMSM = befehl.toFloat();
        Serial.println("Werte übernommen.");

        //Einmaliges Kompensieren des Magnetfelds
        digitalWrite(in1,HIGH); 
        digitalWrite(in2,LOW);
        AusgabeStromMSM.pulse_perc(94.48716002f);
        AusgabeStromLA.pulse_perc(100.00f);
        delay(1000);

        //Einstellen der Lastkraft, Spulen nach wie vor kompensiert, Berechnung Sollstrom LA ca. 6N/A, maximal 100% ED = 724 mA 
        sollStromLA = sollKraftLA/6;
        sollStromLA = constrain(sollStromLA, 0, 0.724); //maximaler Strom wird nicht überschritten
        sollPWMLA = sollStromLA*36,6571359+46,71291013;
        AusgabeStromLA.pulse_perc(sollPWMLA);
        delay (1000);

        //Nullrefernz für den Laser setzen
        nullReferenzLaser = (analogRead(inLaser)*5/1023.0)*7.7/3.98; // 7,7 mm pro 3,98 V
        Serial.print(nullReferenzLaser); Serial.println("mm als Nullreferenz vermessen");
        delay(1000);

        //Um einmaligen Durchlauf des Programmes zu gewährleisten
        i = 0;
        status = Normal;
      }
    }

  if (status == Normal && i == 0){

    //Einstellen Sollmagnetfeld über Sollstrom
    if (sollStromMSM < 0) {
      digitalWrite(in1,LOW);  //Annahme, dass diese Stellung kompensiert
      digitalWrite(in2,HIGH);
      sollPWMMSM =(-sollStromMSM)*48.03281581 + 51.78413903;
      AusgabeStromMSM.pulse_perc(sollPWMMSM);

    }else if(sollStromMSM >= 0){
      digitalWrite(in1,HIGH);  //Annahme, dass diese Stellung kompensiert
      digitalWrite(in2,LOW);
      sollPWMMSM =(sollStromMSM)*48.03281581 + 51.78413903;
      AusgabeStromMSM.pulse_perc(sollPWMMSM);
    }
    
    delay(1000);
    
    long sampleTime_ms = 10;
    unsigned long lastTime_ms = 0;
    unsigned long now = millis();

    float current_pwm = sollPWMMSM;
    
    while (now - lastTime_ms >= sampleTime_ms && lastTime_ms < 2000) {
      lastTime_ms = now

      istStromSP = (analogRead(analogStromSP) * 5.0) / (1023.0 * 1.5);
      
      // PI-Regler
      float error = sollStromMSM - istStromSP;
      integralMSM += (KiMSM * error);
      integralMSM = constrain(integralMSM, -50, 50);
      float pwm_change = (KpMSM * error) + integralMSM
      float pwm_out = current_pwm + pwm_change;
      pwm_out = constrain(pwm_out, 0.0, 100.0);
      
      AusgabeStromMSM.pulse_perc(pwm_out)
      current_pwm = pwm_out;
    }


    istStromSP = (analogRead(analogStromSP) * 5.0) / (1023.0 * 1.5);
    Serial.print("Eingestellter Spulenstrom ist "); Serial.println(istStromSP);
    istStromLA = (analogRead(analogStromLA) * 5.0) / (1023.0 * 1.5); //nur kopiert, ggf. leicht abweichende Parameter
    Serial.print("Eingestellter Lastaktorstrom ist "); Serial.println(istStromLA);

    error = sollStromMSM - istStromSP;
    integralMSM += (KiMSM* error);
    pwm_out = (KpMSM * error) + integralMSM;
    pwm_out = constrain(pwm_out, 0.0, 100.0);
    AusgabeStromMSM.pulse_perc(pwm_out); 
    
    
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
