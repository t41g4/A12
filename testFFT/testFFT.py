import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd

# 日本語フォント設定
matplotlib.rcParams['font.family'] = 'Yu Gothic'

# データ読み込み
df_normal = pd.read_csv('normal.csv')
df_abnormal = pd.read_csv('abnormal.csv')

fs = 100  # サンプリング周波数

def calc_fft(df):
    az = df['az'].values
    az = az - np.mean(az)  # 直流成分を除去
    n = len(az)
    fft_result = np.fft.fft(az)
    frequencies = np.fft.fftfreq(n, d=1/fs)
    amplitude = np.abs(fft_result)
    positive_freq = frequencies[:n//2]
    positive_amp = amplitude[:n//2]
    return positive_freq, positive_amp

freq_n, amp_n = calc_fft(df_normal)
freq_a, amp_a = calc_fft(df_abnormal)

plt.figure(figsize=(12, 6))

# 時系列比較
plt.subplot(2, 1, 1)
plt.plot(df_normal['time_ms'], df_normal['az'], label='正常', alpha=0.7)
plt.plot(df_abnormal['time_ms'], df_abnormal['az'], label='異常', alpha=0.7)
plt.xlabel('時間 (ms)')
plt.ylabel('加速度')
plt.title('振動データ比較（時系列）')
plt.legend()

# FFT比較
plt.subplot(2, 1, 2)
plt.plot(freq_n, amp_n, label='正常', alpha=0.7)
plt.plot(freq_a, amp_a, label='異常', alpha=0.7)
plt.xlabel('周波数 (Hz)')
plt.ylabel('振幅')
plt.title('FFT解析結果比較（直流成分除去済み）')
plt.legend()

plt.tight_layout()
plt.show()