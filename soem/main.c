
#include "ethercat.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <stdbool.h>


// 작업 변수
char IOmap_[4096];
int expectedWkc_;

// PDO 구조체 정의
struct __attribute__((packed)) RxPDO {
    int32_t TargetPosition;      // 0x607A - 목표 위치
    int32_t TargetVelocity;      // 0x60FF - 목표 속도
    int16_t TargetTorque;        // 0x6071 - 목표 토크
    uint8_t ControlWord;         // 0x6040 - 제어 명령
    uint8_t ModesOfOperation;    // 0x6060 - 모드 설정
    uint16_t ImpedancePGain;     // 0x6082 - Impedance P gain
    uint16_t ImpedanceDGain;     // 0x6083 - Impedance D gain
};

struct __attribute__((packed)) TxPDO {
    int32_t PositionActualValue;     // 0x6064 - 실제 위치
    int16_t TorqueActualValue;       // 0x6077 - 실제 토크
    int32_t VelocityActualSensor;    // 0x606C - 실제 속도
    uint16_t Statusword;             // 0x6041 - 상태어
    uint8_t Temperature;             // 0x2031 - 온도
    uint8_t ErrorCode;               // 0x603F - 에러 코드
};

// 게인 및 변환 상수
const double P_GAIN = 100.0;         // 개인
const double D_GAIN = 1.0;           // 개인
const double GEAR_RATIO = 7.15;      // 기어비
const double MOTOR_RESOLUTION = 524288; // 모터 엔코더 해상도 (2^19)

const double RATED_CURRENT = 14.1;   // A 정격 전류
const double TORQUE_CONST = 0.281;   // Nm/A
const double RATED_TORQUE = RATED_CURRENT * TORQUE_CONST; // 3.9621 Nm
const double TORQUE_SCALE = 1000.0;

const double UNIT_TO_DEG = 360.0 / MOTOR_RESOLUTION;  // 0.000686 deg/unit
const double UNIT_TO_RAD = 2 * M_PI / MOTOR_RESOLUTION; // 0.00001198 rad/unit
const double UNIT_TO_JOINT_POS = UNIT_TO_RAD / GEAR_RATIO; // 0.000001675 rad/unit
const double UNIT_TO_MOTOR_POS = 1 / UNIT_TO_RAD; // 596,831.3 unit/rad
const double UNIT_TO_JOINT_VEL = UNIT_TO_JOINT_POS; // same as joint pos
const double UNIT_TO_MOTOR_VEL = 1 / UNIT_TO_JOINT_VEL; // 596,831.3 unit/(rad/s)
const double UNIT_TO_MOTOR_TORQ = RATED_TORQUE * GEAR_RATIO / TORQUE_SCALE; // ≒ 0.02833 Nm
const double UNIT_TO_JOINT_TORQ = 1 / UNIT_TO_MOTOR_TORQ; // 35.3 unit/Nm

// EtherCAT 연결
bool connectEtherCAT(const char* ifname) {
    if (!ec_init(ifname)) {
        printf("No socket connection on %s\n", ifname);
        return false;
    }

    int slaveCount = ec_config_init(false);
    if (slaveCount < 1) {
        printf("No slaves found\n");
        return false;
    }

    printf("Found %d slaves\n", slaveCount);
    return true;
}

// PDO 매핑 설정
bool setupPDOMapping() {
    if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP) {
        printf("Failed to reach PRE_OP state\n");
        return false;
    }

    ec_config_overlap_map(&IOmap_);
    ec_configdc();

    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP) {
        printf("Failed to reach SAFE_OP state\n");
        return false;
    }

    return true;
}



// Operational 상태로 전환
bool waitUntilOperational() {
    expectedWkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Expected work counter: %d\n", expectedWkc_);

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    uint8_t count = 200;
    uint16_t timeout = 50000;
    do {
        ec_send_overlap_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, timeout);
    } while (count-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        printf("Failed to reach OPERATIONAL state\n");
        return false;
    }

    printf("All slaves in OPERATIONAL state\n");
    return true;
}


// Fault Clear
void faultClear() {
    printf("Clearing faults...\n");

    for (uint8_t i = 4; i < ec_slavecount; i++) {
        auto rx = reinterpret_cast<RxPDO *>(ec_slave[i].outputs);
        rx->TargetPosition = 0;
        rx->TargetVelocity = 0;
        rx->TargetTorque = 0;
        rx->ControlWord = 0x0086;         // FAULT_CLEAR
        rx->ModesOfOperation = 0;         // PWM_DUTY_ZERO
        rx->ImpedancePGain = 0;
        rx->ImpedanceDGain = 0;
    }

    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // Fault 제거 확인
    bool noErrorForAllSlaves = false;
    int faultClearCnt = 100;
    do {
        ec_send_overlap_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        noErrorForAllSlaves = true;

        for (uint8_t i = 4; i < ec_slavecount; i++) {
            auto tx = reinterpret_cast<TxPDO *>(ec_slave[i].inputs);
            if (tx->ErrorCode != 0) {
                printf("Slave %d error: %d\n", i, tx->ErrorCode);
                noErrorForAllSlaves = false;
            }
        }
    } while (faultClearCnt-- && !noErrorForAllSlaves);

    if (noErrorForAllSlaves) {
        printf("All faults cleared successfully\n");
    } else {
        printf("Some faults could not be cleared\n");
    }
}



// Impedance Control 모드 설정
void setImpedanceControlMode() {
    printf("Setting Impedance Control mode...\n");

    for (uint8_t i = 4; i < ec_slavecount; i++) {
        auto rx = reinterpret_cast<RxPDO *>(ec_slave[i].outputs);
        rx->ModesOfOperation = 7;     // IMPEDANCE_CONTROL
        rx->ControlWord = 0x000F;     // ENABLE_OPERATION
    }

    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    printf("Impedance Control mode set!\n");
}



// Impedance Control 명령 송신 (raisin과 동일한 단위로 상수 적용)
void setImpedanceControlCommand(
    uint8_t slaveID,
    double targetPos,
    double targetVel,
    double ffTorque)
{
    auto rx = reinterpret_cast<RxPDO *>(ec_slave[slaveID].outputs);

    // raisin과 동일한 단위 변환 적용
    int32_t targetPosMotor = (int32_t)(targetPos * UNIT_TO_MOTOR_POS);   // 라디안 -> 모터 단위
    int32_t targetVelMotor = (int32_t)(targetVel * UNIT_TO_MOTOR_VEL);   // 라디안/초 -> 모터 단위
    int16_t targetTorqueMotor = (int16_t)(ffTorque * UNIT_TO_MOTOR_TOQ); // Nm -> 모터 단위

    // raisin과 동일한 단위로 변환된 P/D 게인
    uint16_t pGainMotor = (uint16_t)(P_GAIN / (GEAR_RATIO * GEAR_RATIO) * 1000.0);
    uint16_t dGainMotor = (uint16_t)(D_GAIN / (GEAR_RATIO * GEAR_RATIO) * 1000.0);

    // 명령 송신
    rx->TargetPosition = targetPosMotor;
    rx->TargetVelocity = targetVelMotor;
    rx->TargetTorque = targetTorqueMotor;
    rx->ImpedancePGain = pGainMotor;
    rx->ImpedanceDGain = 0.196;

    printf("Slave %d - Target: %.3f rad, P: %d, D: %d\n",
           slaveID, targetPos, pGainMotor, dGainMotor);
}


// EtherCAT 통신 사이클
int processEtherCAT() {
    ec_send_overlap_processdata();
    int workCount = ec_receive_processdata(EC_TIMEOUTRET);

    // 슬레이브의 현재 상태 및 값 모니터링
    for (uint8_t i = 4; i < ec_slavecount; i++) {
        auto tx = reinterpret_cast<TxPDO *>(ec_slave[i].inputs);

        // raisin과 동일한 단위 변환
        double actualPos = (double)tx->PositionActualValue * UNIT_TO_JOINT_POS;     // 모터 단위 -> 라디안
        double actualVel = (double)tx->VelocityActualSensor * UNIT_TO_JOINT_VEL;    // 모터 단위 -> 라디안/초
        double actualTorque = (double)tx->TorqueActualValue * UNIT_TO_JOINT_TOQ;    // 모터 단위 -> Nm

        // 상태 워드 확인
        bool isOperational = (tx->Statusword == 39);  // OPERATION_ENABLED

        printf("Slave %d - Pos: %.3f rad, Vel: %.3f rad/s, Torque: %.3f Nm, Status: %s\n",
               i, actualPos, actualVel, actualTorque,
               isOperational ? "OPERATIONAL" : "NOT_OPERATIONAL");
    }

    return workCount;
}



// TODO: 일정 frequency의 제어루프기로 프로그램 짜야함.
int main() {
    printf("=== SOEM Impedance Control Example (P=%.1f, D=%.1f) ===\n", P_GAIN, D_GAIN);
    printf("Motor Resolution: %d\n", (int)MOTOR_RESOLUTION);
    printf("Gear Ratio: %.2f\n", GEAR_RATIO);
    printf("Unit to Motor Pos: %.1f unit/rad\n", UNIT_TO_MOTOR_POS);
    printf("Unit to Motor Vel: %.1f unit/(rad/s)\n", UNIT_TO_MOTOR_VEL);
    printf("Unit to Motor Toq: %.1f unit/Nm\n", UNIT_TO_MOTOR_TOQ);

    // 1. EtherCAT 연결
    printf("\n1. Connecting to EtherCAT...\n");
    if (!connectEtherCAT("eth0")) {
        return -1;
    }

    // 2. PDO 매핑 설정
    printf("\n2. Setting up PDO mapping...\n");
    if (!setupPDOMapping()) {
        return -1;
    }

    // 3. Operational 상태로 전환
    printf("\n3. Transitioning to OPERATIONAL state...\n");
    if (!waitUntilOperational()) {
        return -1;
    }

    // 4. Fault Clear
    printf("\n4. Clearing faults...\n");
    faultClear();

    // 5. Impedance Control 모드 설정
    printf("\n5. Setting Impedance Control mode...\n");
    setImpedanceControlMode();

    printf("\n6. Starting Impedance Control loop...\n");

    // 6. Impedance Control 루프
    int loopCount = 0;
    while (loopCount < 1000) { // 1000번 반복 (약 1초)
        // 각 슬레이브에 대해 Impedance Control 명령 전송
        for (uint8_t i = 4; i < ec_slavecount; i++) {
            // 목표 위치 / 속도 / 피드포워드 토크
            double targetPos = 0.1 * sin(loopCount * 0.01); // 0.1rad 진동
            double targetVel = 0.001 * cos(loopCount * 0.01); // 매우 작은 속도
            double ffTorque = 0.0; // 피드포워드 토크 없음

            setImpedanceControlCommand(i, targetPos, targetVel, ffTorque);
        }

        // EtherCAT 통신 사이클
        int workCount = processEtherCAT();

        // Work counter 확인
        if (workCount < expectedWkc_) {
            printf("Communication error: workCount = %d, expected = %d\n",
                   workCount, expectedWkc_);
        }

        loopCount++;
        usleep(1000); // 1ms 대기 (1kHz 제어 주기)
    }

    // 7. 안전 종료
    printf("\n7. Shutting down safely...\n");

    // 모든 슬레이브를 안전 모드로 전환
    for (uint8_t i = 4; i < ec_slavecount; i++) {
        auto rx = reinterpret_cast<RxPDO *>(ec_slave[i].outputs);
        rx->TargetPosition = 0;
        rx->TargetVelocity = 0;
        rx->TargetTorque = 0;
        rx->ControlWord = 0x0000;         // DISABLE_VOLTAGE
        rx->ModesOfOperation = 0;         // PWM_DUTY_ZERO
        rx->ImpedancePGain = 0;
        rx->ImpedanceDGain = 0;
    }

    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // EtherCAT 종료
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();

    printf("Impedance Control example completed successfully!\n");
    return 0;
}


// TIP See CLion help at <a
// href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>.
//  Also, you can try interactive lessons for CLion by selecting
//  'Help | Learn IDE Features' from the main menu.