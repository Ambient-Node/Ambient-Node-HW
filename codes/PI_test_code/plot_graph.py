import pandas as pd
import matplotlib
matplotlib.use('Agg') # 화면 출력 에러 방지용
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime # [추가] 날짜/시간 라이브러리

# ================= 설정 =================
csv_filename = 'motor_data.csv'
STEP_TO_DEGREE = 0.124
GAP_THRESHOLD = 2000  # 2초 이상 끊기면 분리
# ========================================

# 1. 데이터 읽기
try:
    df = pd.read_csv(csv_filename)
except FileNotFoundError:
    print("CSV 파일이 없습니다.")
    exit()

if df.empty:
    print("데이터가 없습니다.")
    exit()

# 2. 데이터 구간 자동 자르기 (가장 긴 실험 구간 선택)
df['diff'] = df['Time_ms'].diff()
df['new_session'] = df['diff'] > GAP_THRESHOLD
df['session_id'] = df['new_session'].cumsum()

session_counts = df['session_id'].value_counts()
best_session_id = session_counts.idxmax()
best_df = df[df['session_id'] == best_session_id].copy()

print(f"📊 총 {len(session_counts)}개의 실험 구간 중 가장 긴 구간(ID: {best_session_id})을 선택했습니다.")

# 3. 시간축 및 각도 변환
start_time = best_df['Time_ms'].iloc[0]
best_df['Time_sec'] = (best_df['Time_ms'] - start_time) / 1000.0
best_df['Target_Deg'] = best_df['Target_Pos'] * STEP_TO_DEGREE
best_df['Current_Deg'] = best_df['Current_Pos'] * STEP_TO_DEGREE

# 4. 그래프 그리기
plt.figure(figsize=(10, 5))

plt.plot(best_df['Time_sec'], best_df['Target_Deg'], 
         label='Target (Face Position)', 
         color='orange', linestyle='--', linewidth=2)

plt.plot(best_df['Time_sec'], best_df['Current_Deg'], 
         label='Response (Fan Motor)', 
         color='dodgerblue', linewidth=2)

plt.title('System Response Analysis (Face Tracking)', fontsize=14, pad=15)
plt.xlabel('Time (seconds)', fontsize=12)
plt.ylabel('Angle (Degree)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend(fontsize=12)

# 5. [수정] 타임스탬프를 포함하여 파일 저장
# 현재 시간 구하기 (예: 20251128_153000)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_file = f'tracking_result_{timestamp}.png'

plt.tight_layout()
plt.savefig(output_file, dpi=300)

print(f"✅ 그래프 저장 완료: {output_file}")