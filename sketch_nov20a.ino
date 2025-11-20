void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
这就为您修改代码。

### 修改核心思路

为了实现真正的 PI 控制（闭环调节），我做了以下三个关键改变：

1.  **引入反馈机制**：不再使用 `y = kx + d` 这种死板的线性公式。而是实时读取电流传感器数据 (`analogRead`)，计算它与目标值的差值（Error）。
2.  **构建 PI 算法函数**：编写了一个 `computePI()` 函数。
      * **P (比例)**：根据当前误差立即调整 PWM。
      * **I (积分)**：累积过去的误差，用来消除稳态误差（比如单纯 P 控制达到不到设定值的情况）。
3.  **替换 `delay` 为 `调节循环`**：原程序用 `delay(1000)` 等待，这段时间内单片机是“瞎”的。我将其改成了一个持续运行 PI 算法的循环 `maintainCurrent(...)`，在这 1 秒钟内，控制器会不断微调 PWM 以确保电流稳定在设定值。

-----

### 修改后的代码

请注意：**你需要确认电流传感器的转换系数**。我在代码中假设了一个转换公式（基于你原代码中的 `1.5` 系数），如果你的传感器灵敏度不同，请在 `readCurrent` 函数中调整。

```cpp
//Notwendig, um die Frequenz des PWM-Signals verstellen zu können
#include <pwm.h>

//Setzen der Ausgänge für Tauchspulenaktor (Strom) und Spulenstrom
PwmOut AusgabeStromMSM(6);
PwmOut AusgabeStromLA(5);

//Initialisieren der Kanäle
int in1 = 9; 
int in2 = 10;
int in3 = 12; 
int in4 = 13;

//Eingänge der Messgrößen
const int inLaser = A1;
const int analogStromLA = A4;  
const int analogStromSP = A5;

// --- PI REGELUNG VARIABLEN (NEU) ---
// PID Parameter: Diese müssen ggf. getunt (angepasst) werden
float Kp_LA = 2.0;   // Beispielwert: Proportional-Gain
float Ki_LA = 15.0;  // Beispielwert: Integral-Gain (Integral wirkt über Zeit)
float Kp_MSM = 2.0;
float Ki_MSM = 15.0;

float integralErrorLA = 0;
float integralErrorMSM = 0;
unsigned long lastTimeMicro = 0; // Für die Zeitberechnung (dt)

// Globale Variablen für den aktuellen PWM-Wert (damit der Regler nicht bei 0 startet)
float currentPWM_LA = 0;
float currentPWM_MSM = 0;

//Stromregelung Variablen
float sollStromLA = 0;
float sollStromMSM = 0;

//Dehnungsregelung
float istStromSP = 0;
float istLaserSignal = 0;
float nullReferenzLaser = 0;
float auslenkung = 0;
float dehnungMSM = 0;

//Deklaration für Eingabe im Loop:
enum Zustand {Normal, Wartet_W1, Wartet_W2};
Zustand status = Normal;
int i = 0;
float sollKraftLA = 0;


void setup() {
  Serial.begin(9600); 

  //Initialisieren 20 kHz:
  AusgabeStromLA.begin(20000.0f, 0.0f);
  AusgabeStromMSM.begin(20000.0f, 0.0f);

  pinMode(in1,OUTPUT); pinMode(in2,OUTPUT);
  digitalWrite(in1,HIGH); digitalWrite(in2,LOW);

  pinMode(in3,OUTPUT); pinMode(in4,OUTPUT);
  digitalWrite(in3,LOW); digitalWrite(in4,HIGH);
  
  lastTimeMicro = micros(); // Zeit initialisieren
}

// --- HILFSFUNKTION: Strom lesen ---
// Wandelt Analogwert (0-1023) in Ampere um.
// WICHTIG: Prüfen Sie den Faktor (hier Annahme basierend auf altem Code: 1.5V/A ?)
float readCurrent(int pin) {
  float voltage = analogRead(pin) * (5.0 / 1023.0);
  // Wenn 1.5V = 1A, dann Strom = Spannung / 1.5
  // Bitte anpassen an deinen Sensor (z.B. ACS712 hat oft 66mV/A oder 185mV/A)
  return voltage / 1.5; 
}

// --- HILFSFUNKTION: PI-Berechnung ---
// Berechnet den neuen PWM-Wert basierend auf dem Fehler
float computePI(float setpoint, float input, float &integral, float Kp, float Ki, float dt, float currentPWM) {
  float error = setpoint - input;
  
  // Integral Berechnung
  integral += error * dt;
  
  // Anti-Windup (optional aber empfohlen): Begrenzt das Integral, damit es nicht "wegläuft"
  integral = constrain(integral, -100.0, 100.0); 

  // PI Formel: u(t) = Kp*e + Ki*integral
  float outputAdjust = (Kp * error) + (Ki * integral);
  
  // Da PWM ein absoluter Wert ist (0-100%), addieren wir die Regelgröße auf den bestehenden Wert
  // oder wir betrachten den Output direkt als PWM. Hier Ansatz: Direkte Berechnung.
  // Für bessere Stabilität bei PWM oft: PWM_neu = PWM_alt + PI_Output
  float newPWM = (Kp * error) + (Ki * integral); 
  
  // Begrenzung auf 0-100% PWM
  return constrain(newPWM, 0.0f, 100.0f);
}

// --- HILFSFUNKTION: Strom Halten (Regelschleife) ---
// Ersetzt delay(1000). Regelt für 'duration_ms' Millisekunden den Strom aktiv.
void maintainCurrent(float targetLA, float targetMSM, int duration_ms) {
  unsigned long startTime = millis();
  
  // Integral-Reset bei neuem Regelvorgang (optional, verhindert Überschwingen)
  // integralErrorLA = 0; 
  // integralErrorMSM = 0;

  while (millis() - startTime < duration_ms) {
    // Zeitdifferenz berechnen (dt in Sekunden)
    unsigned long now = micros();
    float dt = (now - lastTimeMicro) / 1000000.0;
    lastTimeMicro = now;
    if(dt > 0.1) dt = 0; // Schutz gegen Sprünge beim Start

    // 1. Messen
    float actualLA = readCurrent(analogStromLA);
    float actualMSM = readCurrent(analogStromSP);

    // 2. Rechnen (PI)
    // Wir nutzen hier eine vereinfachte Logik: Der Regler gibt den ABSOLUTEN PWM Wert aus
    // Kp muss hierfür hoch genug sein, oder der Integralanteil übernimmt die Arbeit.
    currentPWM_LA = computePI(targetLA, actualLA, integralErrorLA, Kp_LA, Ki_LA, dt, currentPWM_LA);
    currentPWM_MSM = computePI(targetMSM, actualMSM, integralErrorMSM, Kp_MSM, Ki_MSM, dt, currentPWM_MSM);

    // 3. Stellen (Output)
    // Nur wenn ein Zielstrom gesetzt ist, sonst 0 (Sicherheit)
    if (targetLA > 0.01) AusgabeStromLA.pulse_perc(currentPWM_LA);
    else AusgabeStromLA.pulse_perc(0.0f);

    if (abs(targetMSM) > 0.01) AusgabeStromMSM.pulse_perc(currentPWM_MSM);
    else AusgabeStromMSM.pulse_perc(0.0f);
    
    // Kleines Delay für Stabilität (ADC braucht Zeit)
    delay(1); 
  }
}

void loop() {

  if (Serial.available()) {
      String befehl = Serial.readStringUntil('\n');
      befehl.trim(); 

      if (status == Normal && befehl == "Neu") {
        Serial.println("Konfigurationsmodus: Kraft LA (0 - 8 N)?");
        status = Wartet_W1;
      } else if (status == Wartet_W1) {
        sollKraftLA = befehl.toFloat();
        Serial.println("Stromfaktor MSM (-1.0 bis 1.0)?");
        status = Wartet_W2;
      } else if (status == Wartet_W2) {
        sollStromMSM = befehl.toFloat(); // Dies ist hier ein Faktor, oder? Wenn Ampere, dann ok.
        Serial.println("Starte Sequenz...");

        // 1. Kompensieren (Kalter Start, feste Werte oder Regelung auf Max?)
        // Hier lassen wir es kurz statisch wie früher, oder wir regeln auf Maximalstrom?
        // Bleiben wir beim alten "Pulse" für den Reset-Kick, da PI hier zu langsam sein könnte.
        digitalWrite(in1,HIGH); digitalWrite(in2,LOW);
        AusgabeStromMSM.pulse_perc(94.5f); // Fixwert
        AusgabeStromLA.pulse_perc(100.0f); // Fixwert
        delay(1000);

        // 2. Lastkraft einstellen mit PI REGELUNG
        sollStromLA = sollKraftLA / 6.0; 
        sollStromLA = constrain(sollStromLA, 0, 0.724); 
        
        // Integral Reset für neuen Regelvorgang
        integralErrorLA = 0; 
        
        Serial.print("Regle LA auf Strom: "); Serial.println(sollStromLA);
        // Statt delay(1000) und fester Formel -> 1000ms lang aktiv regeln
        maintainCurrent(sollStromLA, 0, 1500); // 1.5 Sekunden einregeln

        // 3. Nullreferenz messen
        // Währenddessen muss der Strom LA gehalten werden!
        // Wir messen kurz, aber eigentlich müsste der Strom weiterlaufen.
        // maintainCurrent hält den Strom, wir lesen danach.
        nullReferenzLaser = (analogRead(inLaser)*5.0/1023.0)*7.7/3.98; 
        Serial.print(nullReferenzLaser); Serial.println(" mm (Null)");

        i = 0;
        status = Normal;
      }
    }

  if (status == Normal && i == 0){
    
    // Richtung setzen
    if (sollStromMSM < 0) {
      digitalWrite(in1,LOW); digitalWrite(in2,HIGH);
    } else {
      digitalWrite(in1,HIGH); digitalWrite(in2,LOW);
    }
    
    float targetAbsMSM = abs(sollStromMSM); // Annahme: sollStromMSM ist in Ampere
    
    // Reset Integrator für neuen Schritt
    integralErrorMSM = 0;
    
    Serial.println("Regle MSM und LA...");
    // 4. MSM Strom hochfahren und LA halten
    // Wir regeln beide Ströme für 2 Sekunden, damit sie sich stabilisieren
    maintainCurrent(sollStromLA, targetAbsMSM, 2000);

    // Messung
    istStromSP = readCurrent(analogStromSP);
    Serial.print("Gemessener Spulenstrom: "); Serial.println(istStromSP);

    auslenkung = (analogRead(inLaser)*5.0/1023.0)*7.7/3.98; 
    dehnungMSM = abs(((auslenkung - nullReferenzLaser)/15.0)*100.0);

    Serial.println("Kraft, Soll-Strom, Dehnung [%]");
    Serial.print(sollKraftLA); Serial.print(", ");
    Serial.print(sollStromMSM); Serial.print(", ");
    Serial.println(dehnungMSM);
    
    i = 1;
  }  

  // Abschalten
  if (status == Normal && i == 1){
    AusgabeStromMSM.pulse_perc(0.0f);
    AusgabeStromLA.pulse_perc(0.0f);
  }
}
```

-----

### 我是如何实现 PI 调节的？(解读)

#### 1\. 核心公式实现 (在 `computePI` 函数中)

PI 调节器的数学原理被转化为了代码：

$$u(t) = K_p \cdot e(t) + K_i \cdot \int e(t) \, dt$$

  * **`error = setpoint - input;`**: 计算**偏差**（比如：我想让电流是 0.5A，实际只有 0.4A，偏差就是 0.1A）。
  * **`integral += error * dt;`**: 这是**积分项**。`dt` 是时间间隔（单位：秒）。如果偏差一直存在，`integral` 就会越来越大。
  * **`newPWM = (Kp * error) + (Ki * integral);`**: 这是**输出**。
      * 如果误差大，`Kp * error` 会让 PWM 迅速增加。
      * 如果误差很小但还没到目标，`Ki * integral` 会慢慢增加 PWM，直到误差完全消除。

#### 2\. 时间积分 ($dt$) 的引入

原来的代码中没有时间概念。但在积分运算中，时间至关重要。

  * 我使用了 `micros()` 来计算两次循环之间的时间差 `dt`。
  * 这样无论 Arduino 运行得快还是慢，积分的累积物理意义都是正确的（Ampere-Seconds）。

#### 3\. 用主动循环 `maintainCurrent` 替代被动 `delay`

这是程序结构最大的改动。

  * **旧逻辑**：`设置PWM -> 睡1秒 (delay) -> 测量`。在这1秒内，如果电流掉了，单片机不知道。
  * **新逻辑**：`开始循环 -> 读电流 -> 算PI -> 调PWM -> 重复直到1秒结束`。
  * 这样在“等待”稳定的过程中，控制器实际上在疯狂工作（每秒可能调整几百次），确保在测量那一瞬间，电流精准地维持在设定值。

#### 4\. 注意事项 (调试建议)

  * **Kp 和 Ki 参数**：代码中的 `Kp = 2.0, Ki = 15.0` 是经验猜测值。
      * 如果电流**反应太慢**：增加 `Kp`。
      * 如果电流**剧烈震荡**：减小 `Kp`。
      * 如果电流**有稳态误差**（一直差一点点）：增加 `Ki`。
  * **传感器校准**：请务必检查 `readCurrent` 函数里的系数。如果传感器读取的值不对，PI 调节器会把你带到错误的电流值去（因为它非常听话地去追那个错误的读数）。