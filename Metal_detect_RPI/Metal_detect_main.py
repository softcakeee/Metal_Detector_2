import pygame  # 듀얼센스(PS5 컨트롤러)와 상호작용하기 위해 pygame 라이브러리를 임포트
import serial  # UART 시리얼 통신을 사용하기 위해 pySerial 라이브러리를 임포트
import time    # 지연, 타이머 기능 등을 위해 time 라이브러리를 임포트
import matplotlib.pyplot as plt  # 실시간 그래프를 그리기 위해 matplotlib의 pyplot 임포트

# pygame 라이브러리를 초기화하여, 이후에 게임 컨트롤러 등 여러 기능을 사용할 수 있게 준비
pygame.init()

# 컨트롤러(조이스틱) 관련 초기화
pygame.joystick.init()

# 듀얼센스 연결 확인
if pygame.joystick.get_count() == 0:
    # 연결된 조이스틱이 없으면 메시지 출력
    print("DualSense controller is not connected.")
else:
    # 첫 번째(0번) 조이스틱을 선택
    joystick = pygame.joystick.Joystick(0)
    # 선택된 조이스틱 초기화
    joystick.init()
    # 성공적으로 블루투스(무선) 연결이 되었음을 알리는 메시지
    print("DualSense Bluetooth Complete.")

# UART 설정
# 시리얼 포트를 /dev/ttyUSB0로 설정하고, 보드레이트를 9600으로 설정, 타임아웃을 1초로 설정
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  
# 시리얼 통신을 안정화시키기 위해 2초 대기
time.sleep(2)

# 상태 변수 초기화
state_UART = 0  # UART 송신 후 상태(0: 준비, 1: 송신 대기)
Time_state = 0  # 송신 대기시간(타이머) 상태
Left = 3        # 좌회전 명령에 해당하는 숫자(임의 프로토콜 정의)
Right = 2       # 우회전 명령에 해당하는 숫자(임의 프로토콜 정의)
Front = 1       # 전진 명령에 해당하는 숫자(임의 프로토콜 정의)
Back = 0        # 후진 명령에 해당하는 숫자(임의 프로토콜 정의)

# CAP 센서 측정값을 저장할 변수 초기화
CAP_LEFT_VAL = 0 
CAP_MIDDLE_VAL = 0 
CAP_RIGHT_VAL = 0

# 그래프를 그리기 위한 설정
plt.ion()  # interactive 모드 켜기 (실시간 업데이트 가능)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(6,8))  
# subplot 3개 생성(가로 1, 세로 3). 서로 x축을 공유. 크기는 가로 6, 세로 8

# 그래프의 축 라벨 설정
ax1.set_ylabel('CAP_RIGHT_VAL')
ax2.set_ylabel('CAP_MIDDLE_VAL')
ax3.set_ylabel('CAP_LEFT_VAL')
ax3.set_xlabel('Time (s)')

# 각 축에 라인 객체를 생성(처음에는 빈 리스트를 할당)
line_right, = ax1.plot([], [], label='CAP_RIGHT_VAL', color='r')
line_middle, = ax2.plot([], [], label='CAP_MIDDLE_VAL', color='g')
line_left, = ax3.plot([], [], label='CAP_LEFT_VAL', color='b')

# 그래프 제목 설정
ax1.set_title('CAP Sensor Values Over Time (30s window)')

# 범례 추가
ax1.legend()
ax2.legend()
ax3.legend()

# 그래프에 사용할 시간 데이터와 센서값 리스트를 초기화
times = []
cap_right_data = []
cap_middle_data = []
cap_left_data = []

# 프로그램이 시작된 시간을 저장
start_program_time = time.time()

# 그래프에 최근 30초만 표시하기 위한 윈도우 크기(단위: 초)
WINDOW_SIZE = 30  

try:
    while True:
        # pygame 이벤트를 계속해서 받아와서 내부 상태를 갱신
        pygame.event.pump()

        # 조이스틱(듀얼센스) 스틱 값 읽기
        # 왼쪽 스틱의 Y축 값(joystick.get_axis(1))을 -1 ~ 1 범위로 리턴, *1000하여 증폭 후 정수 변환
        LEFT_STICK_Y = round(joystick.get_axis(1) * 1000)
        # 오른쪽 스틱의 X축 값(joystick.get_axis(3))을 -1 ~ 1 범위로 리턴, *1000하여 증폭 후 정수 변환
        RIGHT_STICK_X = round(joystick.get_axis(3) * 1000)

        # state_UART가 0이면 명령을 전송(송신)할 준비 상태
        if state_UART == 0:
            # 왼쪽 스틱 Y값이 음수면 전진(Front), 양수면 후진(Back) 명령 전송
            if LEFT_STICK_Y < 0:
                ser.write(f"{Front}".encode('utf-8'))
                ser.write(f"{abs(LEFT_STICK_Y):04}".encode('utf-8'))
            else:
                ser.write(f"{Back}".encode('utf-8'))
                ser.write(f"{abs(LEFT_STICK_Y):04}".encode('utf-8'))

            # 오른쪽 스틱 X값이 음수면 우회전(Right), 양수면 좌회전(Left) 명령 전송
            if RIGHT_STICK_X < 0:
                ser.write(f"{Right}".encode('utf-8'))
                ser.write(f"{abs(RIGHT_STICK_X):04}\n".encode('utf-8'))
                # 명령 전송 후 state_UART를 1로 바꿔서, 다음 데이터 수신을 대기하도록 함
                state_UART = 1
            else:
                ser.write(f"{Left}".encode('utf-8'))
                ser.write(f"{abs(RIGHT_STICK_X):04}\n".encode('utf-8'))
                # 명령 전송 후 state_UART를 1로 바꿔서, 다음 데이터 수신을 대기하도록 함
                state_UART = 1

        # 시리얼 버퍼에 수신된 데이터가 있는지 확인
        if ser.in_waiting > 0:
            # 한 줄을 읽고, utf-8로 디코딩(오류 발생 시 'replace') 후 양쪽 공백 제거
            data = ser.readline().decode('utf-8', errors='replace').strip()
            # 데이터 포맷이 "CAP_RIGHT_VAL,CAP_MIDDLE_VAL,CAP_LEFT_VAL" 형태라고 가정하고 파싱
            try:
                CAP_RIGHT_VAL, CAP_MIDDLE_VAL, CAP_LEFT_VAL = map(int, data.split(','))
            except ValueError:
                # 데이터 형식이 맞지 않아 파싱에 실패하면 무시하고 다음 루프로
                continue

            # 데이터까지 정상적으로 수신되면 state_UART와 Time_state를 0으로 초기화
            state_UART = 0
            Time_state = 0

            # 그래프에 표시할 값 업데이트
            current_time = time.time() - start_program_time
            times.append(current_time)
            cap_right_data.append(CAP_RIGHT_VAL)
            cap_middle_data.append(CAP_MIDDLE_VAL)
            cap_left_data.append(CAP_LEFT_VAL)

            # 라인 객체에 새로운 데이터 세트(전체 리스트) 할당
            line_right.set_xdata(times)
            line_right.set_ydata(cap_right_data)
            line_middle.set_xdata(times)
            line_middle.set_ydata(cap_middle_data)
            line_left.set_xdata(times)
            line_left.set_ydata(cap_left_data)

            # 최근 30초를 보여주기 위해 x축 범위를 설정
            if current_time > WINDOW_SIZE:
                start_x = current_time - WINDOW_SIZE
            else:
                start_x = 0
            end_x = start_x + WINDOW_SIZE

            # X축 범위를 설정 (sharex=True 이므로 다른 subplot에 동일하게 적용)
            ax1.set_xlim(start_x, end_x)

            # Y축은 현재 표시되는 x 범위 내 데이터에 맞춰 자동 스케일
            ax1.relim()                   # 데이터가 바뀌었음을 알려줌
            ax1.autoscale_view(scalex=False, scaley=True)  
            ax2.relim()
            ax2.autoscale_view(scalex=False, scaley=True)
            ax3.relim()
            ax3.autoscale_view(scalex=False, scaley=True)

            # 그래프 업데이트(화면에 반영)
            plt.draw()
            plt.pause(0.01)

        # 응답 대기 시간 로직: state_UART가 1인 상태에서 1초 간 데이터가 없으면 타임아웃 처리
        if state_UART == 1 and Time_state == 0:
            start_time = time.time()  # 현재 시간 저장
            Time_state = 1
        elif state_UART == 1 and Time_state == 1:
            elapsed_time = time.time() - start_time
            if elapsed_time >= 1:
                # 1초가 지났는데도 응답(데이터)이 없다면 상태 초기화
                state_UART = 0
                Time_state = 0
                print("1sec over")

# 코드가 실행되는 동안 어떤 에러가 발생하면(또는 Ctrl+C로 강제 종료하면) except 블록이 실행됨
except:
    print("코드종료")
    # 종료할 때 모터 등을 정지시킬 명령을 보낼 수도 있음
    ser.write(f"{Front}".encode('utf-8'))
    ser.write(f"{abs(Back):04}".encode('utf-8'))
    ser.write(f"{Right}".encode('utf-8'))
    ser.write(f"{abs(Back):04}\n".encode('utf-8'))
