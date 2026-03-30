/*
 * safe_drive_manual.c
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  수동 주행 명령 -> SafeDrive 출력 연결
 *
 *  처리 문자
 *  ---------------------------------------------------------
 *    F : 전진 100%
 *    Q : 전진 50%
 *    B : 후진 100%
 *    W : 후진 50%
 *    L : 좌회전 100%
 *    E : 좌회전 50%
 *    R : 우회전 100%
 *    T : 우회전 50%
 *    S : 정지
 *
 *  주의
 *  ---------------------------------------------------------
 *  - A / P / D / K 는 app_bms가 처리한다.
 *  - 여기서는 순수하게 "주행 문자"만 처리한다.
 */

#include "safe_drive_manual.h"

#include "safe_drive.h"
#include "direction.h"
#include "speed.h"

/* =========================================================
 * 수동 명령 처리
 * ========================================================= */
uint8_t SafeDriveManual_HandleCmd(uint8_t cmd)
{
    switch (cmd)
    {
        case 'F':
            SafeDrive_Move(CAR_FRONT, SPD_100);
            return 1U;

        case 'Q':
            SafeDrive_Move(CAR_FRONT, SPD_50);
            return 1U;

        case 'B':
            SafeDrive_Move(CAR_BACK, SPD_100);
            return 1U;

        case 'W':
            SafeDrive_Move(CAR_BACK, SPD_50);
            return 1U;

        case 'L':
            SafeDrive_Move(CAR_LEFT, SPD_100);
            return 1U;

        case 'E':
            SafeDrive_Move(CAR_LEFT, SPD_50);
            return 1U;

        case 'R':
            SafeDrive_Move(CAR_RIGHT, SPD_100);
            return 1U;

        case 'T':
            SafeDrive_Move(CAR_RIGHT, SPD_50);
            return 1U;

        case 'S':
            SafeDrive_Stop();
            return 1U;

        default:
            /* 주행 문자가 아니면 처리하지 않음 */
            return 0U;
    }
}
