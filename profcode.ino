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
float KpMSM = 30, KiMSM = 20, KdMSM = 10;
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
//这里有些更改，为了适配python的plot代码
void loop() {
  String befehl = Serial.readStringUntil('\n');
  befehl.trim(); 

  // 1. 接收命令，重置变量
  if (status == Normal && befehl == "Neu") {
    i = 0;
    sollStromMSM = 0.5;
    sollStromLA = 0.5;

    // === 【新增】必须重置积分项，否则第二次运行会直接起飞 ===
    integralLA = 0;
    integralMSM = 0;
  }

  // 2. 开始测量周期
  if (status == Normal && i == 0) {
    
    // 【重要改动 1】在开始测量前，先发送 Python 需要的“暗号”标题
    Serial.println("Kraft, Spulestrom, Dehnung");

    long sampleTime_ms = 10;
    unsigned long duration = 1000;
    unsigned long startTime = millis();
    unsigned long lastSampleTime = millis();
    
    // 进入 2秒 的实时控制与测量循环
    while (millis() - startTime < duration) { // duration = 2000
      unsigned long now = millis();
      
      if(now - lastSampleTime >= sampleTime_ms){
        lastSampleTime = now;
        
        // --- A. 读取与控制 MSM 电流 ---
        istStromSP = (analogRead(analogStromSP) * 5.0) / (1023.0 * 1.5);
        float error = sollStromMSM - istStromSP;
        integralMSM += (KiMSM * error);
        integralMSM = constrain(integralMSM, -100, 100); 
        float pwm_out = (KpMSM * error) + integralMSM;
        pwm_out = constrain(pwm_out, 0.0, 100.0);
        AusgabeStromMSM.pulse_perc(pwm_out);
        current_pwm = pwm_out;

        // --- B. 读取与控制 Lastaktor (力) ---
        istStromLA = (analogRead(analogStromLA) * 5.0) / (1023.0 * 1.5);
        float errorLA = sollStromLA - istStromLA;
        integralLA += (KiLA * errorLA);
        integralLA = constrain(integralLA, -70, 70); 
        float pwm_out_LA = (KpLA * errorLA) + integralLA; 
        pwm_out_LA = constrain(pwm_out_LA, 0.0, 100.0);
        AusgabeStromLA.pulse_perc(pwm_out_LA);

        // --- C. 【重要改动 2】必须在循环内实时计算应变！---
        // 只有放在这里，dehnungMSM 才会随着激光传感器的变化而变化
        auslenkung = (analogRead(inLaser) * 5.0 / 1023.0) * 7.7 / 3.98; 
        dehnungMSM = abs(((auslenkung - nullReferenzLaser) / 15.0) * 100.0);

        // --- D. 发送纯净数据给 Python ---
        // 格式：数值,数值,数值 (对应 Python 的 parts[0], parts[1], parts[2])
        Serial.print(istStromLA); 
        Serial.print(","); 
        Serial.print(istStromSP);
        Serial.print(","); 
        Serial.print(dehnungMSM); // 只有这里换行
        Serial.print(",");          // 加个逗号
        Serial.println(current_pwm); // 打印当前的 PWM 值！
      }
    }

    // 测量结束
    i = 1; 
  }  

  // 停止输出
  if (status == Normal && i == 1) {
    AusgabeStromMSM.pulse_perc(0.0f);
    AusgabeStromLA.pulse_perc(0.0f);
  }
}
