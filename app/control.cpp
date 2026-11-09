/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"
#include "app/arm.h"

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "iwdg.h"

// #include "app/imu_monitor.h"
#include "app/motor_monitor.h"
// #include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

namespace {

enum ArmState_e {
  ARM_CALIBRATE,
  ARM_ENABLE,
  ARM_MANUAL,
  ARM_AUTO
} arm_state = ARM_CALIBRATE;

enum Style {
  RECTANGULAR,
  TRIANGLE,
  STYLE_COUNT  // 左中切换
};

const float path_rectangular[4][2] = {{0.1f,0.1f},{-0.1f,0.1f},{-0.1f,-0.1f},{0.1f,-0.1f}};
//                                    A          B            C             D

// 等边三角形，外接圆半径 0.16：顶点距原点 0.16（最大可达 0.164），
// 必须放大到 0.16，否则直线边会切入不可达内圈导致 trajStart 失败。
const float path_triangle[3][2] = {{0.0f,0.16f},{-0.1386f,-0.08f},{0.1386f,-0.08f}};
//                                  A顶          B左下             C右下

int arm_auto_stage = 0;
// 当前路径选择，左杆MID切换，需要记录上一次的值，否则一直在变
Style arm_auto_style = RECTANGULAR;
bool arm_auto_prev_mid = false;

// 当前样式的路径点数
int pathPointCount(Style s) {
  switch (s) {
    case TRIANGLE:    return 3;
    case RECTANGULAR:
    default:          return 4;
  }
}

// 取当前样式第 i 个点的坐标
void pathPoint(Style s, int i, float& x, float& y) {
  switch (s) {
    case TRIANGLE:    x = path_triangle[i][0];    y = path_triangle[i][1];    break;
    case RECTANGULAR:
    default:          x = path_rectangular[i][0]; y = path_rectangular[i][1]; break;
  }
}
}

#ifdef RF206S
extern WFLY rc;
#elif defined(DR16)
extern RC rc;
#endif

extern Arm arm;
BoardLed led;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool robot_init_flag = false;

// 手操，左上右上
static float rcAxisToJointSpeed(int16_t axis) {
  constexpr int16_t deadband = 20;   
  constexpr int16_t full = 660;      
  constexpr float max_speed = 3.0f;  
  if (axis > deadband) {
    return (axis - deadband) / float(full - deadband) * max_speed;
  }
  if (axis < -deadband) {
    return (axis + deadband) / float(full - deadband) * max_speed;
  }
  return 0.0f;
}

// 控制初始化
void controlInit(void) {
  (&huart3)->Init.BaudRate = 100000;
  (&huart3)->Init.WordLength = UART_WORDLENGTH_9B;
  (&huart3)->Init.StopBits = UART_STOPBITS_1;
  (&huart3)->Init.Parity = UART_PARITY_EVEN;
  (&huart3)->Init.Mode = UART_MODE_RX;
  (&huart3)->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  (&huart3)->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
      Error_Handler();
  }
  led.init();
  robot_state = STOP;
}

// 控制主循环
void controlLoop(void) {
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == decltype(rc)::DOWN);

  if (robot_state == STOP) {
    robotReset();
  } else if (robot_state == STARTUP) {
    robot_init_flag = robotStartup(); // 开机状态判断
  } else if (robot_state == WORKING) {
    allMotorsOn(); //上电
    robotControl();  // 机器人控制
  }
  arm.handle();
  boardLedHandle();
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  robot_init_flag = false;
  arm_state = ARM_CALIBRATE;
  arm_auto_stage = 0;
  arm_auto_style = RECTANGULAR;
  arm_auto_prev_mid = false;
  arm.disable();
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;
  return flag;
}


void robotControl(void) {
  switch (arm_state) {
    case ARM_CALIBRATE:
      // 电机上电然后伸直标定
      if (arm.calibrate(0.0f, 0.0f)) {
        arm_state = ARM_ENABLE;
      }
      break;

    case ARM_ENABLE:
      arm_auto_prev_mid = false;
      if (rc.switch_.r == decltype(rc)::UP) {
        if (rc.switch_.l == decltype(rc)::UP && arm.enable()) {
          arm_state = ARM_MANUAL;
        } else if (rc.switch_.l == decltype(rc)::MID ||
                   rc.switch_.l == decltype(rc)::DOWN) {
          arm_state = ARM_AUTO;
        }
      }
      break;

    case ARM_MANUAL:
      arm_auto_prev_mid = false;
      if (rc.switch_.l != decltype(rc)::UP) {
        arm_state = ARM_ENABLE;  //
      } else {
        // 右摇杆上下  J0 速度，左摇杆上下  J1 速度
        arm.addJointReference(rcAxisToJointSpeed(rc.channel_.r_row),
                              rcAxisToJointSpeed(rc.channel_.l_row), 0.001f);
      }
      break;

    case ARM_AUTO: {
      if (rc.switch_.l == decltype(rc)::UP) {
        arm_state = ARM_ENABLE;  // 左杆 UP，退回使能
        break;
      }
      // 左杆 MID 切换到下一条路径
      const bool l_mid = (rc.switch_.l == decltype(rc)::MID);
      if (l_mid && !arm_auto_prev_mid) {
        arm_auto_style = static_cast<Style>((arm_auto_style + 1) % STYLE_COUNT);
        arm_auto_stage = 0;
        arm.trajAbort();
      }
      arm_auto_prev_mid = l_mid;

      const float side_time = 0.8f;
      const int n = pathPointCount(arm_auto_style);
      if (arm_auto_stage == 0) {
        float x, y, q[2];
        pathPoint(arm_auto_style, 0, x, y);
        CartesianReference node;
        node.x = x; node.y = y;
        if (arm.enable() && ArmKinematics::inverse(node, arm.state().q, 1, q) &&
            arm.trajStartJoint(q[0], q[1], side_time)) {
          arm_auto_stage = 1;
        }
      }
      else if (arm_auto_stage == 1) {
        if (!arm.trajRunning()) {
          arm_auto_stage = 2;
        }
      }
      else {
        if (!arm.trajRunning()) {
          const int idx = (arm_auto_stage - 1) % n;
          float x, y;
          pathPoint(arm_auto_style, idx, x, y);
          if (arm.trajStart(x, y, side_time)) {
            arm_auto_stage = (arm_auto_stage >= n + 1) ? 2 : arm_auto_stage + 1;
          }
        }
      }
      break;
    }
  }
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  }
  else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  }
  else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
    if (arm_state == ARM_CALIBRATE) {
      led.setModeBlink(1);  // 标定中
    }
    else if (arm_state == ARM_ENABLE) {
      led.setModeBlink(2);  // 使能中
    }
    else if (arm_state == ARM_MANUAL){
      led.setColor(100,100,100);
      led.setModeBlink(3);  // 遥控手动
    }
    else { //ARM_AUTO
      led.setColor(150,150,150);
      led.setModeBlink(4);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (arm_state == ARM_CALIBRATE) {
      led.setLED(false, true, 1);  // green
    } else if (arm_state == ARM_ENABLE) {
      led.setLED(false, true, 2);  // green
    } else {
      led.setLED(false, true, 3);  // green
    }
  }
#endif
}
