import os
import glob
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import griddata
import re

# 配置路径 (请根据你的实际情况修改) / Pfadkonfiguration
FOLDER_PATH = r"C:\Users\16771\Desktop\2025_2026冬季学期\PROJEKT\新建文件夹\Automatischer_Modus_mit_Array\MSM_Dehnung\CSV_1次（间隔0.2A）"


def build_interactive_everett_map(folder_path):
    file_paths = glob.glob(os.path.join(folder_path, "forc_*.csv"))
    if not file_paths:
        print("❌ 未找到文件！")
        return

    alpha_list, beta_list, E_list = [], [], []

    for file in file_paths:
        filename = os.path.basename(file)
        match = re.search(r"forc_([-+]?\d*\.?\d+)A\.csv", filename)
        if not match: continue
        alpha = float(match.group(1))

        df = pd.read_csv(file)
        curr_arr = df['Current'].values
        strain_arr = df['Strain'].values

        # 找到反转点 (Current 最大值)
        peak_idx = np.argmax(curr_arr)
        f_alpha = strain_arr[peak_idx]

        desc_currents = curr_arr[peak_idx:]
        desc_strains = strain_arr[peak_idx:]

        for beta, f_alpha_beta in zip(desc_currents, desc_strains):
            # 逻辑修正：针对特定点的特殊处理
            # Logik-Korrektur: Sonderbehandlung für den spezifischen Punkt
            if abs(alpha - 1.0) < 1e-6 and abs(beta - (-1.0)) < 1e-6:
                E = f_alpha - f_alpha_beta  # 不乘以 0.5
            else:
                E = 0.5 * (f_alpha - f_alpha_beta)  # 保持 0.5

            alpha_list.append(alpha)
            beta_list.append(beta)
            E_list.append(E)

    # 插值与绘图 / Interpolation und Plotten
    ai = np.linspace(-1.0, 1.0, 100)
    bi = np.linspace(-1.0, 1.0, 100)
    AI, BI = np.meshgrid(ai, bi)

    ZI = griddata((alpha_list, beta_list), E_list, (AI, BI), method='linear')
    ZI[BI > AI] = None

    fig = go.Figure(data=[go.Surface(
        z=ZI, x=AI, y=BI,
        colorscale='Viridis',
        colorbar_title='Everett E (%)'
    )])

    fig.update_layout(
        title='Interactive 3D Everett Map (Customized Scaling)',
        scene=dict(
            xaxis=dict(title='Alpha (Reversal A)', range=[-1.0, 1.0]),
            yaxis=dict(title='Beta (Descending A)', range=[-1.0, 1.0]),
            zaxis=dict(title='E (%)'),
            camera=dict(eye=dict(x=-1.5, y=-1.5, z=1.2))
        )
    )

    fig.show()


if __name__ == "__main__":
    build_interactive_everett_map(FOLDER_PATH)