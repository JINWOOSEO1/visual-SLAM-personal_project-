#!/usr/bin/env python3
"""
test_motor_encoder.py
왼쪽 모터 → 왼쪽 엔코더, 오른쪽 모터 → 오른쪽 엔코더 순으로
직접 구동하며 누적 틱을 출력하는 스탠드얼론 테스트 스크립트.

사용법:
    sudo python3 test_motor_encoder.py
    (ros2 데몬과 독립적으로 동작. encoder_node 실행 중이면 GPIO 충돌하므로 종료 후 실행)
"""

import time
import RPi.GPIO as GPIO

# ─── 핀 정의 (CLAUDE.md 기준) ─────────────────────────────────
# L298N - 왼쪽 모터
ENA = 24
IN1 = 22
IN2 = 23
# L298N - 오른쪽 모터
ENB = 25
IN3 = 27
IN4 = 26
# 엔코더
LEFT_ENC  = 17
RIGHT_ENC = 16

# ─── 테스트 파라미터 ──────────────────────────────────────────
DURATION    = 3.0   # 각 바퀴 회전 시간 (초)
DUTY_CYCLE  = 70    # PWM duty (%)
PWM_FREQ    = 1000  # PWM 주파수 (Hz)

# ─── 틱 카운터 ────────────────────────────────────────────────
left_ticks  = 0
right_ticks = 0


def cb_left(channel):
    global left_ticks
    left_ticks += 1


def cb_right(channel):
    global right_ticks
    right_ticks += 1


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # 모터 핀
    for pin in (ENA, IN1, IN2, ENB, IN3, IN4):
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    # 엔코더 핀
    GPIO.setup(LEFT_ENC,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENC, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENC,  GPIO.BOTH, callback=cb_left,  bouncetime=1)
    GPIO.add_event_detect(RIGHT_ENC, GPIO.BOTH, callback=cb_right, bouncetime=1)

    pwm_a = GPIO.PWM(ENA, PWM_FREQ)
    pwm_b = GPIO.PWM(ENB, PWM_FREQ)
    pwm_a.start(0)
    pwm_b.start(0)
    return pwm_a, pwm_b


def drive_left(pwm_a, duty):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(duty)


def stop_left(pwm_a):
    pwm_a.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)


def drive_right(pwm_b, duty):
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_b.ChangeDutyCycle(duty)


def stop_right(pwm_b):
    pwm_b.ChangeDutyCycle(0)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


def run_phase(name, start_fn, stop_fn, get_ticks):
    global left_ticks, right_ticks
    left_ticks = 0
    right_ticks = 0

    print(f'\n=== {name} 구동 시작 ({DURATION}s @ duty={DUTY_CYCLE}%) ===')
    start_fn()
    t0 = time.time()
    while time.time() - t0 < DURATION:
        elapsed = time.time() - t0
        print(f'  [{elapsed:4.1f}s] left={left_ticks:5d}  right={right_ticks:5d}',
              end='\r', flush=True)
        time.sleep(0.1)
    stop_fn()
    time.sleep(0.3)  # 관성 정지 대기

    print(f'\n--- {name} 결과 ---')
    print(f'  왼쪽 엔코더  누적: {left_ticks}')
    print(f'  오른쪽 엔코더 누적: {right_ticks}')
    print(f'  (기대: {name} 엔코더만 증가해야 함)')


def main():
    pwm_a, pwm_b = setup()
    try:
        run_phase(
            '왼쪽 바퀴',
            lambda: drive_left(pwm_a, DUTY_CYCLE),
            lambda: stop_left(pwm_a),
            lambda: (left_ticks, right_ticks),
        )
        time.sleep(1.0)
        run_phase(
            '오른쪽 바퀴',
            lambda: drive_right(pwm_b, DUTY_CYCLE),
            lambda: stop_right(pwm_b),
            lambda: (left_ticks, right_ticks),
        )
    except KeyboardInterrupt:
        print('\n[중단됨]')
    finally:
        stop_left(pwm_a)
        stop_right(pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        print('\nGPIO 정리 완료.')


if __name__ == '__main__':
    main()
