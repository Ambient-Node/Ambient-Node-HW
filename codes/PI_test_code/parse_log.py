import csv

# 파일 이름 설정
input_filename = 'log_dump.txt'     # 복사한 로그를 저장한 파일
output_filename = 'motor_data.csv'  # 결과로 나올 CSV 파일

print(f"변환 시작: {input_filename} -> {output_filename}")

count = 0

with open(input_filename, 'r', encoding='utf-8') as infile, \
     open(output_filename, 'w', newline='', encoding='utf-8') as outfile:
    
    writer = csv.writer(outfile)
    # CSV 헤더 작성 (시간, 목표, 현재)
    writer.writerow(['Time_ms', 'Target_Pos', 'Current_Pos'])
    
    for line in infile:
        # 1. 데이터 수신 마커 확인 (⬅️)
        if "[UART] ⬅️" in line:
            try:
                # 2. 화살표 뒤의 내용만 추출
                # 예: "ambient-fan-service | [UART] ⬅️ 40994,-52,-51"
                # split 결과: ["ambient...", " 40994,-52,-51"]
                content = line.split("[UART] ⬅️")[1].strip()
                
                # 3. 콤마로 분리
                values = content.split(',')
                
                # 4. 데이터가 3개이고 모두 숫자인지 확인 (Graph Mode OFF 같은 문자열 제외)
                if len(values) == 3:
                    # 정수로 변환 가능한지 테스트
                    int(values[0])
                    int(values[1])
                    int(values[2])
                    
                    # CSV 쓰기
                    writer.writerow(values)
                    count += 1
            except (ValueError, IndexError):
                # 숫자가 아니거나 형식이 깨진 줄은 조용히 넘어감
                continue

print(f"변환 완료! 총 {count}개의 유효 데이터를 추출했습니다.")