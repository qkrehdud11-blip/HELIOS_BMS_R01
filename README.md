### STM32-Based Solar Charging, Battery Safety, and RC Car Control Platform

---

## 📌 Project Overview

**HELIOS-BMS**는 STM32 MCU를 기반으로 제작한 **태양광 충전**, **배터리 안전 감지**, **RC Car 주행 제어**를 통합한 임베디드 시스템입니다.

시스템은 크게 3개의 보드로 구성됩니다.

- **Remote** : 조이스틱 및 버튼 입력을 통해 사용자 명령 전송
- **Solar Master Board** : 차량 주행 제어, BMS 상태 판단, CAN 통신 제어
- **Solar Slave Board** : 태양광 추적 및 Buck Converter 기반 충전 제어

본 프로젝트는 **태양광 에너지 활용**, **배터리 상태 기반 안전 제어**, **수동/자율 주행 기능**, **Master-Slave CAN 통신 구조**를 하나의 플랫폼으로 통합하는 것을 목표로 합니다.

---

## 🎯 Project Goal

- STM32 기반 통합 EV/RC 플랫폼 구현  
- 태양광 기반 배터리 충전 시스템 구현  
- 광원 추적을 통한 태양광 수집 효율 향상  
- 배터리 온도/가스/전압/전류 기반 안전 감지 구현  
- 수동 주행 및 자율 주행 기능 구현  
- Master-Slave 구조 기반 CAN 통신 제어 구현  
- 상태 머신(State Machine) 기반 구조 설계  

---

## 🔑 Key Features

### ☀️ Solar Charging System
- Buck Converter 기반 배터리 충전 회로
- INA219 기반 전압/전류 측정
- **MPPT / CC / CV 통합 충전 제어**
- PI 제어 기반 PWM duty 제어
- 충전 상태 머신 기반 안정적 동작

### 🔦 Solar Tracking System
- CDS 조도 센서를 이용한 태양 방향 감지
- Pan / Tilt 서보 모터 제어
- 태양광 입사각 최적화

### 🔋 Battery Safety Management
- 온도, 가스, 전압, 전류 센서 기반 상태 감지
- SAFE / WARNING / DANGER 상태 분류
- 위험 수준에 따른 속도 제한
- Danger 발생 시 강제 정지

### 🚗 Driving Control
- 조이스틱 기반 **Manual Mode**
- 초음파 센서 기반 **Auto Mode**
- 장애물 감지 및 회피 주행
- 안전 상태 기반 주행 제한

### 📡 Communication
- Bluetooth(UART) 기반 Remote 제어
- CAN 기반 Master ↔ Slave 통신
- 제어 명령 및 상태 데이터 송수신

---

## 🧠 System Architecture

```text
[ Remote Controller ]
  - Joystick
  - Buttons
  - Bluetooth UART
          │
          ▼
[ Solar Master Board ]
  - Driving Control
  - BMS Monitoring
  - Safety Manager
  - CAN Control
          │
          ▼
[ Solar Slave Board ]
  - Solar Tracking
  - Solar Sensing
  - Charger Control
  - PWM Duty Control
```
---

## 🔄 Operating Concept

### 1. Remote
- 조이스틱 ADC 입력 처리  
- 버튼 입력 기반 모드 전환  
- Bluetooth UART를 통해 Master에 명령 전송  

---

### 2. Solar Master Board
- Remote 명령 수신  
- Manual / Auto 상태 머신 실행  
- 배터리 상태 실시간 모니터링  
- 안전 상태에 따른 속도 제한 및 정지 수행  
- CAN을 통해 Slave Board 제어  

---

### 3. Solar Slave Board
- CAN 명령 수신  
- 태양광 추적 시스템 제어  
- INA219 기반 전력 측정  
- MPPT / CC / CV 충전 제어  
- PWM Duty 제어  

---

## ⚙️ Software Structure

### 📁 Repository Layout

    HELIOS_BMS_R01/
    ├── Remote/
    ├── Solar_master_R01/
    └── Solar_slave_R01/

### 🔹 Solar Master
- `app_bms` : 상위 통합 제어
- `bms_sensor` : 센서 데이터 수집
- `bms_safety_manager` : 안전 상태 판단
- `statemachine` : 주행 상태 머신
- `safe_drive` : 속도 제한 적용
- `can` : CAN 통신

### 🔹 Solar Slave
- `app_charger` : 충전 시스템 제어
- `solar_sensing` : INA219 센싱
- `solar_pi_control` : PI 제어
- `charger_state` : 충전 상태 머신

### 🔹 Remote
- 조이스틱 ADC 입력 처리
- 버튼 입력 처리
- UART 기반 Bluetooth 송신

---

## 🔄 Main Control Logic

### 🔹 Master Side

    App_Bms_Init()
     └─ Sensor Init
     └─ Safety Init
     └─ StateMachine Init
     └─ CAN Init

    App_Bms_Task()
     ├─ Sensor Task
     ├─ Driving StateMachine
     ├─ Safety Manager
     ├─ Danger Check
     └─ CAN Process

### 🔹 Slave Side

    STMACHINE_Init()
     └─ Charger Init
     └─ CAN Init

    ST_MACHINE()
     └─ Charger / Tracking Control

    App_Charger
     ├─ Solar Sensing
     ├─ Charger State
     ├─ PI Control
     └─ PWM Apply

---

## 🚘 Driving Modes

### 🕹️ Manual Mode
- 조이스틱 기반 차량 제어
- Bluetooth 명령 수신
- PWM 기반 모터 제어

### 🤖 Auto Mode
- 초음파 센서 기반 장애물 감지
- 회피 및 재탐색 수행
- 위험 상태 시 정지

---

## 🔋 Charging Modes

- **MPPT** : 최대 전력점 추종
- **CC** : 정전류 충전
- **CV** : 정전압 충전
- **STOP** : 충전 종료

---

## 🛡️ Safety Policy

- Warning 개수 기반 속도 제한
- Danger 발생 시 즉시 정지
- Force Stop 상태 유지
- 재초기화를 통한 시스템 복구

---

## ⚙️ Tech Stack

- **Language**: C
- **MCU**: STM32F411
- **Framework**: STM32 HAL
- **IDE**: STM32CubeIDE
- **Communication**: UART, CAN, I2C, SPI
- **Sensor**: INA219, CDS, Ultrasonic, Temperature, Gas
- **Control**: PWM, PI Control
- **Version Control**: Git, GitHub

---

## 🔩 Hardware Components

- STM32F411 Black Pill
- INA219 Sensor
- CDS Sensor
- Ultrasonic Sensor
- Gas Sensor
- Temperature Sensor
- Servo Motor
- DC Motor & Driver
- Bluetooth Module
- CAN Module
- Solar Panel
- Li-ion Battery

---

## 📌 Highlights

- 태양광 충전 + 안전 + 주행 제어 통합 시스템
- Master / Slave / Remote 구조 설계
- 상태 머신 기반 안정적 제어 구조
- CAN 통신 기반 확장성 확보

---

## 🚀 Future Work

- CAN 프로토콜 정리
- 로그 시각화 시스템
- 안전 상태 기록 기능
- UI 대시보드 구현

---

## 👨‍💻 Author

**Park Doyoung**  
Embedded Systems / Power Electronics / Control / Firmware
