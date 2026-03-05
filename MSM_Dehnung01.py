# import serial
# import time
# import matplotlib.pyplot as plt
# import pandas as pd
#
# # --- Port Configuration ---
# SERIAL_PORT = 'COM3'  # Please change this to your actual port
# BAUD_RATE = 9600
#
#
# def collect_real_data():
#     currents = []
#     strains = []
#
#     print(f"1. Connecting to serial port {SERIAL_PORT}...")
#     try:
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
#         time.sleep(2)  # Wait for Arduino to reset
#     except Exception as e:
#         print(f"Serial Port Error: {e}")
#         return [], []
#
#     # Send command
#     print("2. Sending start command 'Neu'...")
#     ser.write(b'Neu\n')
#
#     print("3. Waiting for data transmission...")
#     recording = False
#
#     try:
#         while True:
#             line = ser.readline().decode('utf-8', errors='ignore').strip()
#             if not line: continue
#
#             # Print raw data to monitor sensor activity
#             print(f"[Arduino]: {line}")
#
#             if line == "START_DATA":
#                 recording = True
#                 currents = []
#                 strains = []
#                 print(">>> Recording started...")
#                 continue
#
#             if line == "END_DATA":
#                 print(">>> Recording finished")
#                 break
#
#             if recording:
#                 parts = line.split(',')
#                 # Compatible format: [Force, Current, Strain, PWM_LA, PWM_MSM]
#                 if len(parts) >= 3:
#                     try:
#                         c = float(parts[1])
#                         s = float(parts[2])
#                         currents.append(c)
#                         strains.append(s)
#                     except ValueError:
#                         pass
#     except KeyboardInterrupt:
#         print("\nUser interrupted")
#     finally:
#         ser.close()
#
#     return currents, strains
#
#
# def plot_two_colors(currents, strains):
#     if not currents:
#         print("No data collected, unable to plot")
#         return
#
#     print("4. Generating plot...")
#
#     # --- Key Algorithm: Finding the inflection point ---
#     # Find the index of the maximum current (1.0A)
#     # Assuming the data sequence is -1 -> ... -> 1 -> ... -> -1
#     max_val = max(currents)
#     peak_idx = currents.index(max_val)
#
#     # --- Splitting Data ---
#     # 1. Forward Path (Red): From start to peak (inclusive)
#     curr_up = currents[:peak_idx + 1]
#     strain_up = strains[:peak_idx + 1]
#
#     # 2. Return Path (Blue): From peak to end
#     curr_down = currents[peak_idx:]
#     strain_down = strains[peak_idx:]
#
#     # --- Plotting ---
#     plt.figure(figsize=(10, 6))
#
#     # Plot Forward Path (Loading)
#     plt.plot(curr_up, strain_up, color='red', marker='o', linestyle='-',
#              linewidth=2, label='Loading (-1A $\\to$ 1A)')
#
#     # Plot Return Path (Unloading)
#     plt.plot(curr_down, strain_down, color='blue', marker='x', linestyle='--',
#              linewidth=2, label='Unloading (1A $\\to$ -1A)')
#
#     # Add direction arrows (Auxiliary)
#     if len(curr_up) > 1:
#         mid = len(curr_up) // 2
#         plt.annotate('', xy=(curr_up[mid], strain_up[mid]), xytext=(curr_up[mid - 1], strain_up[mid - 1]),
#                      arrowprops=dict(arrowstyle='->', color='red', lw=2))
#
#     if len(curr_down) > 1:
#         mid = len(curr_down) // 2
#         plt.annotate('', xy=(curr_down[mid], strain_down[mid]), xytext=(curr_down[mid - 1], strain_down[mid - 1]),
#                      arrowprops=dict(arrowstyle='->', color='blue', lw=2))
#
#     # Styling
#     plt.title('MSM Actuator Hysteresis Loop', fontsize=14)
#     plt.xlabel('Current (A)', fontsize=12)
#     plt.ylabel('Displacement / Strain (%)', fontsize=12)
#     plt.grid(True, linestyle='--', alpha=0.6)
#     plt.legend()
#     plt.show()
#
#
# if __name__ == "__main__":
#     # 1. Data acquisition
#     c, s = collect_real_data()
#
#     # 2. Visualization
#     plot_two_colors(c, s)

import serial
import time
import matplotlib.pyplot as plt
import pandas as pd

# --- Port Configuration ---
SERIAL_PORT = 'COM3'  # Please change this to your actual port
BAUD_RATE = 9600


def collect_real_data():
    currents = []
    strains = []

    print(f"1. Connecting to serial port {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)  # Wait for Arduino to reset
    except Exception as e:
        print(f"Serial Port Error: {e}")
        return [], []

    # Send command
    print("2. Sending start command 'Neu'...")
    ser.write(b'Neu\n')

    print("3. Waiting for data transmission...")
    recording = False

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue

            # Print raw data to monitor sensor activity
            print(f"[Arduino]: {line}")

            if line == "START_DATA":
                recording = True
                currents = []
                strains = []
                print(">>> Recording started...")
                continue

            if line == "END_DATA":
                print(">>> Recording finished")
                break

            if recording:
                parts = line.split(',')
                # Compatible format: [Force, Current, Strain, PWM_LA, PWM_MSM]
                if len(parts) >= 3:
                    try:
                        c = float(parts[1])
                        s = float(parts[2])
                        currents.append(c)
                        strains.append(s)
                    except ValueError:
                        pass
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        ser.close()

    return currents, strains


def plot_two_colors(currents, strains):
    if not currents:
        print("No data collected, unable to plot")
        return

    print("4. Generating plot...")

    # --- Key Algorithm: Finding the inflection point ---
    # Find the index of the maximum current (1.0A)
    # Assuming the data sequence is -1 -> ... -> 1 -> ... -> -1
    max_val = max(currents)
    peak_idx = currents.index(max_val)

    # --- Splitting Data ---
    # 1. Forward Path (Red): From start to peak (inclusive)
    curr_up = currents[:peak_idx + 1]
    strain_up = strains[:peak_idx + 1]

    # 2. Return Path (Blue): From peak to end
    curr_down = currents[peak_idx:]
    strain_down = strains[peak_idx:]

    # --- Plotting ---
    plt.figure(figsize=(10, 6))

    # Plot Forward Path (Loading)
    plt.plot(curr_up, strain_up, color='red', marker='o', linestyle='-',
             linewidth=2, label='Loading (-1A $\\to$ 1A)')

    # Plot Return Path (Unloading)
    plt.plot(curr_down, strain_down, color='blue', marker='x', linestyle='--',
             linewidth=2, label='Unloading (1A $\\to$ -1A)')

    # Add direction arrows (Auxiliary)
    if len(curr_up) > 1:
        mid = len(curr_up) // 2
        plt.annotate('', xy=(curr_up[mid], strain_up[mid]), xytext=(curr_up[mid - 1], strain_up[mid - 1]),
                     arrowprops=dict(arrowstyle='->', color='red', lw=2))

    if len(curr_down) > 1:
        mid = len(curr_down) // 2
        plt.annotate('', xy=(curr_down[mid], strain_down[mid]), xytext=(curr_down[mid - 1], strain_down[mid - 1]),
                     arrowprops=dict(arrowstyle='->', color='blue', lw=2))

    # Styling
    plt.title('MSM Actuator Hysteresis Loop', fontsize=14)
    plt.xlabel('Current (A)', fontsize=12)
    plt.ylabel('Displacement / Strain (%)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # ========================================================
    # 每次测试前，请在这里手动修改你要保存的文件名！
    # ========================================================
    csv_filename = 'forc_-0.8A.csv'

    # 1. Data acquisition
    c, s = collect_real_data()

    # 2. Save data to CSV (新增的导出功能)
    if c and s:
        df = pd.DataFrame({
            'Current': c,
            'Strain': s
        })
        df.to_csv(csv_filename, index=False)
        print(f"\n✅ 成功: 数据已自动保存至当前文件夹下的 {csv_filename} 文件中。")
    else:
        print("\n❌ 警告: 没有收集到有效数据，未生成 CSV 文件。")

    # 3. Visualization
    plot_two_colors(c, s)