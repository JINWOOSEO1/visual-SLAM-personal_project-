# RC Car Visual SLAM

라즈베리파이 4B 기반 차동구동(differential drive) RC카의 ROS 2 워크스페이스.
단안 카메라 + 휠 엔코더 + IMU를 EKF로 융합해 오도메트리를 만들고,
PC 쪽에서 RTAB-Map으로 2D Visual SLAM을 돌린다.

무거운 SLAM 연산은 PC가 맡고 Pi는 센서 퍼블리시와 모터 제어만 담당하는
분산 구조다. 둘은 같은 Wi-Fi의 ROS 2 DDS로 통신한다.

```
┌─────────────────── Raspberry Pi 4B ────────────────────┐      ┌──────── PC ────────┐
│                                                        │      │                    │
│  IMX219 CSI ──▶ camera_ros ──▶ /camera/image_raw       │      │                    │
│                             └▶ /camera/image_raw/      │─Wi-Fi▶  image_transport   │
│                                        compressed      │      │     republish      │
│                                                        │      │         │          │
│  MPU6050 ──▶ mpu6050_node ──▶ /imu/data_raw            │      │         ▼          │
│   (I2C)             │                                  │      │     rtabmap        │
│                     ▼                                  │      │   (2D SLAM, ORB)   │
│              imu_kalman_node ──▶ /imu/data             │      │         │          │
│                                    │                   │      │         ▼          │
│  엔코더 ──▶ encoder_node ──▶ /encoder/ticks            │      │   TF: map → odom   │
│  (GPIO)            │                                   │      │  /rtabmap/cloud_map│
│                    ▼                                   │      └────────────────────┘
│             odometry_node ──▶ /odom                    │
│                                    │                   │
│                    ┌───────────────┴────┐              │
│                    ▼                    ▼              │
│               robot_localization EKF ──▶ /odometry/filtered
│                                          TF: odom → base_link
│                                                        │
│  /cmd_vel ──▶ pid_controller_node ──▶ L298N ──▶ 모터   │
│               (또는 motor_driver_node)                 │
└────────────────────────────────────────────────────────┘
```

## 하드웨어

| 항목 | 사양 |
|---|---|
| 보드 | Raspberry Pi 4B 4GB, Ubuntu 22.04, ROS 2 Humble |
| 카메라 | IMX219 (Arducam), CSI 연결, libcamera + 하드웨어 ISP |
| IMU | MPU6050 (GY-521), I2C bus 1 @ `0x68` |
| 모터 드라이버 | L298N |
| 엔코더 | 단채널 광학 엔코더 ×2 (방향 정보 없음, 펄스만 카운트) |
| 휠 | 직경 65 mm, 휠베이스 150 mm, 40 펄스/회전 (BOTH 엣지) |

### GPIO 핀맵 (BCM)

| 기능 | 핀 |
|---|---|
| 좌 모터 ENA / IN1 / IN2 | GPIO24 / GPIO22 / GPIO23 |
| 우 모터 ENB / IN3 / IN4 | GPIO25 / GPIO27 / GPIO26 |
| 좌 엔코더 | GPIO17 |
| 우 엔코더 | GPIO16 |

> L298N IN 핀은 좌·우 모두 코드 기준과 반대로 배선되어 있다.
> 그래서 `left_invert` / `right_invert` 가 둘 다 `true` 로 설정돼 있다.

### TF 프레임

```
map ──(rtabmap)──▶ odom ──(EKF)──▶ base_link ──┬──▶ camera_link ──▶ camera
                                               └──▶ imu_link
```

`base_link` 원점은 뒷바퀴 축 중심. 오프셋은 `rc_car_bringup/launch/sensor_tf.launch.py` 참고.
IMU는 x축이 로봇 우측, y축이 전방을 향하도록 장착되어 있어 `yaw = -π/2` 가 걸려 있다.

## 패키지

### `mpu6050_driver`
MPU6050을 I2C로 직접 읽어 IMU 데이터를 퍼블리시한다.

- **`mpu6050_node`** → `/imu/data_raw` (50 Hz), `/imu/temperature`
  - 기동 시 정지 상태에서 자이로 바이어스를 매번 다시 추정한다 (온도에 따라 변하므로).
  - 가속도계는 `config/imu_calibration.yaml` 의 계수로 보정한다:
    `a_corr = (a_raw - bias) * scale`
  - 기동 시 정지 상태 `|a|` 를 9.807 m/s² 와 비교하는 헬스체크를 출력한다.
- **`imu_kalman_node`** → `/imu/data`
  - 2-state 칼만 필터(각도 + 자이로 바이어스)로 roll/pitch를 추정해 orientation을 채운다.
  - 지자기 센서가 없어 yaw는 자이로 적분이라 드리프트한다. EKF가 `differential` 모드로 받아 변화량만 쓴다.
- **`scripts/calibrate_accel.py`** — 가속도계 6면 캘리브레이션 ([아래](#가속도계-캘리브레이션) 참고)

### `wheel_odometry`
- **`encoder_node`** → `/encoder/ticks` (`Int32MultiArray [left, right]`, 50 Hz)
  - GPIO 인터럽트로 양쪽 엣지를 센다. **부호 없는 누적 카운트**만 퍼블리시한다.
- **`odometry_node`** → `/odom`
  - 차동구동 기구학으로 위치를 적분한다.
  - 엔코더에 방향 정보가 없어서 `/cmd_vel` 의 부호로 각 바퀴의 진행 방향을 추론한다.
    즉 **명령과 반대로 밀리면(슬립·외력) 오도메트리가 그 방향을 잘못 안다.**
  - TF는 EKF가 broadcast하므로 기본적으로 `publish_tf: false`.

### `pid_velocity_controller` (권장 — closed-loop)
- **`pid_controller_node`** — `/cmd_vel` + `/encoder/ticks` → PID → L298N PWM
  - 피드포워드(측정한 duty–속도 1차 모델) + PID 보정 구조.
  - 방향이 바뀌면 적분항을 리셋한다.
  - `/cmd_vel` 이 `cmd_timeout`(0.5 s) 동안 안 오면 정지 — Wi-Fi 끊김 대비.
- **`step_cmd_vel_node`** — 스텝 응답 시험용 `/cmd_vel` 퍼블리셔
- **`analyze_pid_step_response`** — 컨트롤러가 남긴 CSV 로그에서 상승시간·오버슈트 등 지표 계산

### `motor_controller` (대안 — open-loop)
- **`motor_driver_node`** — `/cmd_vel` 을 PWM duty로 직접 매핑. 엔코더 피드백 없음.

> ⚠️ **`pid_controller_node` 와 `motor_driver_node` 를 동시에 실행하면 안 된다.**
> 같은 GPIO 핀(ENA/ENB/IN1–4)을 제어하므로 둘 중 하나만 골라 쓴다.

### `rc_car_bringup`
런치 파일과 설정 모음.

| 런치 | 실행 위치 | 내용 |
|---|---|---|
| `sensors.launch.py` | Pi | 카메라 + IMU + 엔코더/오도메트리 + static TF + EKF (모터 제외) |
| `ekf.launch.py` | Pi | `robot_localization` EKF 단독 |
| `sensor_tf.launch.py` | Pi | `base_link` → `camera_link` / `camera` / `imu_link` static TF |
| `rtabmap.launch.py` | **PC** | 이미지 decompress + RTAB-Map + 뷰어 |
| `teleop.launch.py` | 아무데나 | `teleop_twist_keyboard` |

## 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 을 쓰면 Python 파일과 config YAML을 고쳐도 재빌드가 필요 없다.
(`setup.py` / `package.xml` / 런치 파일을 바꿨을 때는 다시 빌드해야 한다.)

### 의존 패키지

```bash
sudo apt install ros-humble-camera-ros ros-humble-robot-localization \
                 ros-humble-image-transport ros-humble-teleop-twist-keyboard
# PC 측에만 필요
sudo apt install ros-humble-rtabmap-slam ros-humble-rtabmap-viz
```

Python: `RPi.GPIO`, `smbus`, `PyYAML`

## 실행

### 1. Pi — 센서 스택

```bash
ros2 launch rc_car_bringup sensors.launch.py
```

### 2. Pi — 모터 제어 (별도 터미널, 둘 중 하나만)

```bash
ros2 launch pid_velocity_controller pid_controller.launch.py   # closed-loop (권장)
# 또는
ros2 launch motor_controller motor.launch.py                   # open-loop
```

### 3. PC — SLAM

```bash
ros2 launch rc_car_bringup rtabmap.launch.py
ros2 launch rc_car_bringup rtabmap.launch.py rtabmap_viz:=false   # GUI 없이
```

Wi-Fi 대역폭 때문에 raw 이미지는 17 fps까지 떨어져서, Pi는 compressed만 보내고
PC가 `image_transport/republish` 로 풀어서 RTAB-Map에 넣는다.

### 4. 조종

```bash
ros2 launch rc_car_bringup teleop.launch.py
```

> `teleop_twist_keyboard` 는 현재 Pi에 설치돼 있지 않다. 먼저 설치해야 한다:
> `sudo apt install ros-humble-teleop-twist-keyboard`

## 캘리브레이션

### 가속도계 캘리브레이션

MPU6050(특히 저가 GY-521 클론)은 가속도 감도 오차가 10%를 넘는 개체가 흔하다.
이 저장소의 개체는 정지 상태에서 `|a| = 11.16 m/s²` (정상 9.807, **+13.9%**)를 읽는다.
`ACCEL_CONFIG` 레지스터는 정상이므로 설정 문제가 아니라 센서 개체 오차다.

평평하고 단단한 책상 위에서:

```bash
python3 src/mpu6050_driver/scripts/calibrate_accel.py
```

6개 자세(X위/X아래/Y위/Y아래/Z위/Z아래)를 안내대로 잡아주면 축별 bias와 scale을 푼다:

```
bias  = (r_up + r_dn) / 2
scale = 2g / (r_up - r_dn)
```

결과는 `src/mpu6050_driver/config/imu_calibration.yaml` 에 저장되고,
다음 `mpu6050_node` 기동 때 자동으로 적용된다. 로그에 이렇게 뜨면 정상이다:

```
[INFO] [mpu6050_node]: Accel correction: bias=[...], scale=[...]
[INFO] [mpu6050_node]: Gravity check OK: |a|=9.8xx m/s²
```

보정을 안 했으면 `Gravity check FAILED` 경고가 뜬다.

> **참고:** 현재 EKF는 가속도(`ax/ay/az`)를 융합에서 제외하고 있어서(`ekf.yaml`의
> `imu0_config`) 이 오차가 위치 추정에 직접 새어 들어가지는 않는다. 다만
> `imu_kalman_node` 가 중력벡터로 roll/pitch를 계산하므로 **축별 오프셋**은
> 자세 편향(약 2–4°)으로 나타난다. 균일 스케일 오차는 `atan2` 비율에서 상쇄된다.

### 카메라 내부 파라미터

```bash
ros2 launch intrinsic_calibration.launch.py
```

7×5 체커보드(사각형 25 mm) 기준. 결과는 `~/.ros/camera_info/` 에 저장된다.

### 엔코더 / 하드웨어 점검

`~/hw_check/` 에 배선 점검용 스탠드얼론 스크립트가 있다 (저장소 밖).

```bash
python3 ~/hw_check/test_encoder_manual.py   # 바퀴를 손으로 돌리며 틱 확인
python3 ~/hw_check/test_imu_motion.py       # IMU를 기울이며 축별 반응 확인
```

## 센서 융합 설정

`rc_car_bringup/config/ekf.yaml` (`two_d_mode: true`):

| 소스 | 사용하는 상태 | 이유 |
|---|---|---|
| `/odom` (휠) | `vx`, `vy`, `vyaw` | pose는 슬립이 누적되므로 twist만 사용. `vy=true` 는 비홀로노믹 제약(옆방향 속도 0) |
| `/imu/data` | `yaw`, `vyaw` (differential) | 절대 방위 기준이 없으므로 변화량만 사용 |
| 가속도 | **사용 안 함** | RC카 진동으로 노이즈가 크고, 이중적분은 드리프트가 심함 |

## 알려진 이슈

- **엔코더에 방향 정보가 없다.** 단채널이라 `/cmd_vel` 부호로 방향을 추론한다.
  외력이나 슬립으로 명령과 반대로 움직이면 오도메트리가 틀린다. 정확도가 더 필요하면
  쿼드러처(2채널) 엔코더로 바꿔야 한다.
- **yaw 드리프트.** 지자기 센서가 없어 yaw는 자이로 적분이다. RTAB-Map의 루프
  클로저가 `map → odom` 에서 보정하지만, 특징점이 부족한 구간에서는 누적된다.
- **문서와 코드 불일치.** 일부 주석과 `rc_car_bringup/package.xml` 이 orientation
  필터로 `imu_filter_madgwick` 을 언급하지만, 실제로 실행되는 것은
  `mpu6050_driver/imu_kalman_node` 다.
- **`/dev/media*` 번호는 재부팅마다 바뀐다.** libcamera가 알아서 찾으므로 보통
  문제되지 않지만, `media-ctl` 을 직접 쓸 때는 `unicam` + `imx219` 문자열로 탐색해야 한다.

## 트러블슈팅

**IMU가 안 잡힘**
```bash
i2cdetect -y 1        # 0x68 이 보여야 정상
```

**카메라가 안 잡힘**
```bash
ros2 run camera_ros camera_node   # 로그에 imx219 가 나오는지 확인
ros2 topic hz /camera/image_raw   # 640x480 RGB888 에서 약 23 Hz
```
`cam -l` (libcamera CLI)은 이 환경에서 세그폴트한다. ROS 노드 경로는 정상이니 무시해도 된다.

**모터가 안 움직임**
`pid_controller_node` 와 `motor_driver_node` 가 동시에 떠 있는지 먼저 확인한다.
GPIO 충돌이 가장 흔한 원인이다.

```bash
ps aux | grep -E "pid_controller_node|motor_driver_node"
```

**노드를 껐는데 토픽이 계속 나옴**
`ros2 run` 을 백그라운드로 띄웠다 죽이면 자식 프로세스가 남는 경우가 있다.

```bash
pkill -f mpu6050_node
pkill -f camera_node
```
