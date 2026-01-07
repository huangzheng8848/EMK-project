//为了实现任务 2（负载执行器仿真复位弹簧），需要原有的 PI 电流控制循环中，将原本固定的 sollStromLA（目标电流）改为根据实时位移计算的动态值。
// --- 1. 在变量声明区补充 (约第 35 行后) ---在 变量声明区域之后，以及 while (millis() - startTime < 2000) 循环内部进行如下补充：-----------

float k_feder = 2.5;           // 虚拟弹簧刚度 (N/mm)，需根据实验调节 [1]
float current_pos = 0;         // 当前实时位移 (mm) [2]
float delta_x = 0;             // 相对于零点的位移量 (mm) [3]

// --- 2. 在 loop() 的 while 采样循环内补充 (约第 85 行) ---
while (millis() - startTime < 2000) {
    unsigned long now = millis();
    if (now - lastSampleTime >= sampleTime_ms) {
        lastSampleTime = now;

        // ============ 核心补充：弹簧仿真逻辑==============
        // A. 读取实时位移 (使用来源提供的转换系数)---------------------------------
        current_pos = (analogRead(inLaser) * 5.0 / 1023.0) * 7.7 / 3.98; // [2, 4]
        delta_x = current_pos - nullReferenzLaser; // 计算压缩量 [3]

        // B. 计算模拟弹簧力 F = k * delta_x，并转换为目标电流, 使用负载执行器 VM33xx-180 的力常数 6 N/A [5, 6]
        sollStromLA = (k_feder * delta_x) / 6.0f; 
        
        // C. 安全限制：确保电流不超过持续负载上限 724mA [4-6]---------------------
        sollStromLA = constrain(sollStromLA, 0.0, 0.724); 

        // --- 接入原有的 PI 控制逻辑 -------------------------------
        istStromLA = (analogRead(analogStromLA) * 5.0) / (1023.0 * 1.5); // [7, 8]
        float errorLA = sollStromLA - istStromLA;
        integralLA += (KiLA * errorLA);
        integralLA = constrain(integralLA, -70, 70); // Anti-Windup [8]
        
        float pwm_out_LA = (KpLA * errorLA) + integralLA; // [9]
        pwm_out_LA = constrain(pwm_out_LA, 0.0, 100.0);
        AusgabeStromLA.pulse_perc(pwm_out_LA); 

        // ... (保留 MSM 的控制逻辑) ...
    }
}