/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序 - 集成电机控制、PID、RPLIDAR、MPU6500（含优化自动转向功能）
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms in LICENSE file or provided AS-IS.
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float integral_limit;
} PID_Controller;

/* Private define ------------------------------------------------------------*/
#define MAX_SPEED 400
#define MAX_SPEED_RIGHT 406
#define SPEED_UPDATE_INTERVAL 10
#define TURN_DURATION 800
#define TURN_SPEED 300
#define DIR_STOP 0
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_AUTO_LEFT 5  // 自动左转90度
#define DIR_AUTO_RIGHT 6 // 自动右转90度
#define PPR 360
#define SAMPLE_TIME_MS 100
#define M_PI 3.14159265358979323846
#define DMA_BUFFER_SIZE 256
#define DATA_PACKET_SIZE 5
#define ANGLE_FILTER_THRESHOLD 1.0f
#define MIN_VALID_DISTANCE 50.0f
#define MAX_VALID_DISTANCE 12000.0f
#define LIDAR_TIMEOUT_THRESHOLD 500

// PID参数
#define PID_KP 1.2f
#define PID_KI 0.1f
#define PID_KD 0.05f
#define PID_INTEGRAL_LIMIT 200.0f
#define RPM_TO_PWM_FACTOR 2.0f

// MPU6500配置
#define MPU6500_ADDR 0xD0
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75
#define CALIBRATION_SAMPLES 500
#define ACCEL_FS_SEL_4G 0x08
#define GYRO_FS_SEL_500DPS 0x08

// 自动转向参数
#define TURN_ANGLE_THRESHOLD 3.0f      // 角度容差阈值(±3度)
#define AUTO_TURN_SPEED 300            // 自动转向速度
#define MIN_TURN_SPEED 120             // 最小转向速度
#define SLOWDOWN_ANGLE 45.0f           // 开始减速的角度
#define OVERSHOOT_THRESHOLD 30.0f      // 过冲检测阈值

// 零偏校准参数
#define GYRO_BIAS_UPDATE_INTERVAL 10   // 零偏更新间隔(ms)
#define COMPLEMENTARY_FILTER_GAIN 0.02f // 互补滤波器增益
#define STATIONARY_THRESHOLD 0.1f      // 静止检测阈值(m/s²)
#define STATIONARY_DURATION 1000       // 静止持续时间(ms)

/* Private variables ---------------------------------------------------------*/
uint8_t bluetooth_rx_data = 0;
uint8_t target_direction = DIR_STOP;
uint8_t current_direction = DIR_STOP;
uint32_t current_speed = 0;
uint32_t target_speed = 0;
uint32_t last_speed_update_time = 0;

uint8_t bluetooth_connected = 0;
uint8_t connection_announced = 0;
uint8_t connection_msg[] = "Connected\r\n";

int32_t lastEncoderA = 0;
int32_t lastEncoderB = 0;

char uart_buf[256];

uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];
uint8_t lidar_dataBuffer[DATA_PACKET_SIZE];
uint8_t lidar_dataIndex = 0;
uint32_t lidar_rxIndex = 0;
volatile uint8_t lidar_process_packet = 0;
float lastLidarAngle = 0.0f;
uint32_t lastLidarRxTime = 0;

int16_t accel_offset_x, accel_offset_y, accel_offset_z;
int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temp;
float roll, pitch;
float yaw = 0.0f;  // 偏航角
uint32_t last_imu_time = 0;  // 用于计算时间间隔

PID_Controller pid_left = { PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, PID_INTEGRAL_LIMIT };
PID_Controller pid_right = { PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, PID_INTEGRAL_LIMIT };
float target_rpm = 0.0f;
float pwm_left = 0, pwm_right = 0;

// 自动转向控制变量
uint8_t auto_turn_mode = 0;        // 0:非自动转向 1:左转90度 2:右转90度
float start_yaw = 0.0f;            // 开始转向时的初始偏航角
float target_yaw = 0.0f;           // 目标偏航角
float prev_angle_diff = 0.0f;      // 上一次角度差（用于检测过冲）
uint8_t overshoot_count = 0;       // 过冲计数器

// 零偏校准变量
float gyro_z_bias = 0.0f;          // 陀螺仪Z轴零偏
uint8_t is_stationary = 0;         // 是否静止标志
uint32_t stationary_start_time = 0;// 静止开始时间

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ControlMotor(uint8_t direction, uint32_t speed);
void UpdateSpeedRamp(void);
void SendConnectionNotification(void);
void ProcessLidarData(uint8_t* data);
void SendLidarToBluetooth(float angle, float distance, uint8_t quality);
float PID_Update(PID_Controller* pid, float setpoint, float measurement);
void MPU6500_Init(I2C_HandleTypeDef* hi2c);
void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c);
void MPU6500_Read_All(I2C_HandleTypeDef* hi2c);
void Calculate_Angles(void);
void Send_IMU_Data_Bluetooth(void);
float CalculateAngleDifference(float current, float target);
void HandleAutoTurn(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  计算角度差值（考虑360度边界）
  * @param  current: 当前角度
  * @param  target: 目标角度
  * @retval 角度差值
  */
float CalculateAngleDifference(float current, float target) {
    float diff = target - current;

    // 处理360度边界情况
    if (diff > 180.0f) {
        diff -= 360.0f;
    }
    else if (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

/**
  * @brief  PID控制器更新
  * @param  pid: PID控制器结构体
  * @param  setpoint: 目标值
  * @param  measurement: 测量值
  * @retval PID输出值
  */
float PID_Update(PID_Controller* pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

/**
  * @brief  UART接收完成回调
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        if (!bluetooth_connected) bluetooth_connected = 1;
        if (bluetooth_rx_data >= '0' && bluetooth_rx_data <= '7') {  // 扩展到'7'
            target_direction = bluetooth_rx_data - '0';

            // 处理自动转向指令
            if (target_direction == DIR_AUTO_LEFT || target_direction == DIR_AUTO_RIGHT) {
                auto_turn_mode = target_direction;  // 设置自动转向模式
                start_yaw = yaw;  // 记录当前偏航角
                prev_angle_diff = 0.0f;
                overshoot_count = 0;

                // 计算目标偏航角（使用相对角度）
                if (target_direction == DIR_AUTO_LEFT) {
                    target_yaw = yaw + 90.0f;
                    if (target_yaw >= 360.0f) target_yaw -= 360.0f;
                }
                else {
                    target_yaw = yaw - 90.0f;
                    if (target_yaw < 0.0f) target_yaw += 360.0f;
                }

                // 设置转向速度
                current_speed = AUTO_TURN_SPEED;
                target_speed = AUTO_TURN_SPEED;

                // 发送确认信息
                char msg[60];
                snprintf(msg, sizeof(msg), "Auto turn started. Start:%.1f°, Target:%.1f°\r\n", start_yaw, target_yaw);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
            // 处理其他方向指令
            else {
                auto_turn_mode = 0;  // 退出自动转向模式

                switch (target_direction) {
                case DIR_STOP:
                    target_speed = 0;
                    current_speed = 0;
                    break;
                case DIR_FORWARD:
                case DIR_BACKWARD:
                    target_speed = MAX_SPEED;
                    current_speed = MAX_SPEED;
                    break;
                case DIR_LEFT:
                case DIR_RIGHT:
                    target_speed = TURN_SPEED;
                    current_speed = TURN_SPEED;
                    break;
                }
                ControlMotor(target_direction, current_speed);
            }
        }
        HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);
    }
    else if (huart == &huart6) {
        HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);
        if (lidar_dma_buffer[0] == 0xA5) {
            lidar_process_packet = 1;
        }
    }
}

/**
  * @brief  电机控制函数
  * @param  direction: 方向
  * @param  speed: 速度
  * @retval None
  */
void ControlMotor(uint8_t direction, uint32_t speed) {
    switch (direction) {
    case DIR_STOP:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_FORWARD:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)pwm_left);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)pwm_right);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_BACKWARD:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm_left);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)pwm_right);
        break;
    case DIR_LEFT:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_RIGHT:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, TURN_SPEED);
        break;
    }
}

/**
  * @brief  速度斜坡更新
  * @retval None
  */
void UpdateSpeedRamp(void) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        last_speed_update_time = current_time;

        // 自动转向模式下不处理速度斜坡
        if (auto_turn_mode != 0) return;

        if (target_direction == DIR_STOP && current_direction != DIR_STOP) {
            current_speed = 0;
            current_direction = DIR_STOP;
            ControlMotor(DIR_STOP, 0);
            return;
        }

        if (target_direction != current_direction) {
            current_direction = target_direction;
            if (current_direction == DIR_LEFT || current_direction == DIR_RIGHT) {
                current_speed = TURN_SPEED;
                ControlMotor(current_direction, current_speed);
            }
            else if (current_direction != DIR_STOP) {
                current_speed = target_speed = MAX_SPEED;
                ControlMotor(current_direction, current_speed);
            }
        }
        else if (current_direction != DIR_STOP) {
            if (current_speed < target_speed) {
                current_speed = target_speed;
                ControlMotor(current_direction, current_speed);
            }
        }
    }
}

/**
  * @brief  发送连接通知
  * @retval None
  */
void SendConnectionNotification(void) {
    if (bluetooth_connected && !connection_announced) {
        HAL_UART_Transmit(&huart1, connection_msg, sizeof(connection_msg) - 1, 1000);
        connection_announced = 1;
    }
}

/**
  * @brief  处理自动转向
  * @retval None
  */
void HandleAutoTurn(void) {
    // 计算当前角度与目标角度的差值
    float angle_diff = CalculateAngleDifference(yaw, target_yaw);
    float abs_diff = fabsf(angle_diff);

    // 检测过冲（方向反转）
    if ((angle_diff * prev_angle_diff < 0) && (abs_diff > OVERSHOOT_THRESHOLD)) {
        overshoot_count++;
        if (overshoot_count > 2) {
            // 多次过冲，强制停止
            ControlMotor(DIR_STOP, 0);
            auto_turn_mode = 0;
            overshoot_count = 0;

            // 根据过冲角度更新零偏
            gyro_z_bias += angle_diff * 0.01f;

            // 发送通知
            char msg[60];
            snprintf(msg, sizeof(msg), "Auto turn overshoot! Stopped. Adjusted bias: %.3f\r\n", gyro_z_bias);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            return;
        }
    }
    else {
        overshoot_count = 0;
    }
    prev_angle_diff = angle_diff;

    // 检查是否达到目标角度
    if (abs_diff <= TURN_ANGLE_THRESHOLD) {
        // 完成转向
        ControlMotor(DIR_STOP, 0);
        auto_turn_mode = 0;

        // 转向完成后校准零偏
        float current_gyro = gyro_z / 65.5f;
        gyro_z_bias = 0.95f * gyro_z_bias + 0.05f * current_gyro;

        HAL_UART_Transmit(&huart1, (uint8_t*)"Auto turn completed!\r\n", 22, 100);
    }
    else {
        // 智能调速：距离目标越近速度越慢
        float speed_factor = 1.0f;
        if (abs_diff < SLOWDOWN_ANGLE) {
            speed_factor = 0.3f + 0.7f * (abs_diff / SLOWDOWN_ANGLE);
        }

        uint32_t adjusted_speed = (uint32_t)(AUTO_TURN_SPEED * speed_factor);
        if (adjusted_speed < MIN_TURN_SPEED) adjusted_speed = MIN_TURN_SPEED;

        // 应用转向
        if (auto_turn_mode == DIR_AUTO_LEFT) {
            // 左转：右轮前进，左轮后退
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, adjusted_speed); // 左后退
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, adjusted_speed); // 右前进
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        }
        else if (auto_turn_mode == DIR_AUTO_RIGHT) {
            // 右转：左轮前进，右轮后退
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, adjusted_speed); // 左前进
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, adjusted_speed); // 右后退
        }

        // 调试信息
        static uint32_t last_debug_time = 0;
        if (HAL_GetTick() - last_debug_time > 200) {
            last_debug_time = HAL_GetTick();
            snprintf(uart_buf, sizeof(uart_buf),
                "AutoTurn: Mode:%d, Cur:%.1f, Tgt:%.1f, Diff:%.1f, Spd:%d, OS:%d, Bias:%.3f\n",
                auto_turn_mode, yaw, target_yaw, angle_diff, adjusted_speed, overshoot_count, gyro_z_bias);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
        }
    }
}

/**
  * @brief  处理激光雷达数据
  * @param  data: 数据指针
  * @retval None
  */
void ProcessLidarData(uint8_t* data) {
    typedef struct {
        uint8_t sync_quality;
        uint16_t angle_q6;
        uint16_t distance_q2;
    } __attribute__((packed)) LidarDataPacket;

    LidarDataPacket* pkt = (LidarDataPacket*)data;

    if ((pkt->sync_quality & 0x80) && (pkt->angle_q6 & 0x01)) {
        uint8_t quality = pkt->sync_quality & 0x7F;
        float angle = (pkt->angle_q6 >> 1) / 64.0f;
        float distance = pkt->distance_q2 / 4.0f;

        if (distance >= MIN_VALID_DISTANCE && distance <= MAX_VALID_DISTANCE && quality > 0) {
            if (fabsf(angle - lastLidarAngle) >= ANGLE_FILTER_THRESHOLD || angle < lastLidarAngle) {
                SendLidarToBluetooth(angle, distance, quality);
                lastLidarAngle = angle;
            }
        }
    }

    if (lidar_process_packet) {
        lastLidarRxTime = HAL_GetTick();
        __HAL_DMA_DISABLE(huart6.hdmarx);
        huart6.hdmarx->Instance->NDTR = DMA_BUFFER_SIZE;
        __HAL_DMA_ENABLE(huart6.hdmarx);
        lidar_process_packet = 0;
    }
}

/**
  * @brief  发送激光雷达数据到蓝牙
  * @param  angle: 角度
  * @param  distance: 距离
  * @param  quality: 质量
  * @retval None
  */
void SendLidarToBluetooth(float angle, float distance, uint8_t quality) {
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "A:%.2f,D:%.2f,Q:%d\n", angle, distance, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
}

/**
  * @brief  MPU6500初始化
  * @param  hi2c: I2C句柄
  * @retval None
  */
void MPU6500_Init(I2C_HandleTypeDef* hi2c) {
    uint8_t check, data;

    // 检查设备ID
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, WHO_AM_I, 1, &check, 1, 100);
    if (check == 0x70) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6500 Found!\r\n", 16, 100);
    }
    else {
        char msg[30];
        snprintf(msg, sizeof(msg), "MPU6500 Not Found! ID:0x%X\r\n", check);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        Error_Handler();
    }

    // 唤醒设备
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    HAL_Delay(100);

    // 配置陀螺仪 ±500°/s
    data = GYRO_FS_SEL_500DPS;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, GYRO_CONFIG, 1, &data, 1, 100);

    // 配置加速度计 ±4g
    data = ACCEL_FS_SEL_4G;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, ACCEL_CONFIG, 1, &data, 1, 100);

    // 配置低通滤波器 5Hz
    data = 0x06;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, CONFIG, 1, &data, 1, 100);

    // 采样率 1kHz/(1+7)=125Hz
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, SMPLRT_DIV, 1, &data, 1, 100);
}

/**
  * @brief  MPU6500校准
  * @param  hi2c: I2C句柄
  * @retval None
  */
void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c) {
    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Calibrating MPU6500...\r\n", 24, 100);

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        uint8_t buf[14];
        HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

        // 累加原始数据
        accel_sum_x += (int16_t)((buf[0] << 8) | buf[1]);
        accel_sum_y += (int16_t)((buf[2] << 8) | buf[3]);
        accel_sum_z += (int16_t)((buf[4] << 8) | buf[5]);
        gyro_sum_x += (int16_t)((buf[8] << 8) | buf[9]);
        gyro_sum_y += (int16_t)((buf[10] << 8) | buf[11]);
        gyro_sum_z += (int16_t)((buf[12] << 8) | buf[13]);

        HAL_Delay(5);
    }

    // 计算平均值作为偏移
    accel_offset_x = accel_sum_x / CALIBRATION_SAMPLES;
    accel_offset_y = accel_sum_y / CALIBRATION_SAMPLES;
    accel_offset_z = (accel_sum_z / CALIBRATION_SAMPLES) - 16384; // 1g对应值

    gyro_offset_x = gyro_sum_x / CALIBRATION_SAMPLES;
    gyro_offset_y = gyro_sum_y / CALIBRATION_SAMPLES;
    gyro_offset_z = gyro_sum_z / CALIBRATION_SAMPLES;

    // 打印校准结果
    char cal_msg[128];
    snprintf(cal_msg, sizeof(cal_msg),
        "Calibration Complete:\r\n"
        "Accel Offsets: X:%d Y:%d Z:%d\r\n"
        "Gyro Offsets: X:%d Y:%d Z:%d\r\n",
        accel_offset_x, accel_offset_y, accel_offset_z,
        gyro_offset_x, gyro_offset_y, gyro_offset_z);
    HAL_UART_Transmit(&huart1, (uint8_t*)cal_msg, strlen(cal_msg), 100);
}

/**
  * @brief  读取MPU6500所有数据
  * @param  hi2c: I2C句柄
  * @retval None
  */
void MPU6500_Read_All(I2C_HandleTypeDef* hi2c) {
    uint8_t buf[14];

    // 读取14字节数据 (加速度+温度+陀螺仪)
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

    // 减去校准偏移量
    accel_x = (int16_t)((buf[0] << 8) | buf[1]) - accel_offset_x;
    accel_y = (int16_t)((buf[2] << 8) | buf[3]) - accel_offset_y;
    accel_z = (int16_t)((buf[4] << 8) | buf[5]) - accel_offset_z;

    temp = (int16_t)((buf[6] << 8) | buf[7]);

    gyro_x = (int16_t)((buf[8] << 8) | buf[9]) - gyro_offset_x;
    gyro_y = (int16_t)((buf[10] << 8) | buf[11]) - gyro_offset_y;
    gyro_z = (int16_t)((buf[12] << 8) | buf[13]) - gyro_offset_z;
}

/**
  * @brief  计算角度（含偏航角） - 优化版
  * @retval None
  */
void Calculate_Angles(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 计算时间差（秒）
    float delta_time = (current_time - last_time) / 1000.0f;
    if (delta_time > 0.2f) delta_time = 0.01f; // 限制最大时间差
    last_time = current_time;

    // 转换为g单位 (±4g量程, 8192 LSB/g)
    float ax = accel_x / 8192.0f;
    float ay = accel_y / 8192.0f;
    float az = accel_z / 8192.0f;

    // 计算加速度幅值（用于静止检测）
    float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);

    // 检测是否静止（重力加速度≈1g）
    if (fabsf(accel_magnitude - 1.0f) < STATIONARY_THRESHOLD) {
        if (is_stationary == 0) {
            is_stationary = 1;
            stationary_start_time = current_time;
        }
        // 长时间静止后更新零偏
        else if (current_time - stationary_start_time > STATIONARY_DURATION) {
            // 低通滤波更新零偏
            float current_bias = gyro_z / 65.5f;
            gyro_z_bias = 0.98f * gyro_z_bias + 0.02f * current_bias;
        }
    }
    else {
        is_stationary = 0;
    }

    // 计算横滚角和俯仰角
    roll = atan2f(ay, az) * 180.0f / M_PI;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    // 计算偏航角（陀螺仪积分），减去零偏
    float gyro_z_dps = (gyro_z / 65.5f) - gyro_z_bias;
    yaw += gyro_z_dps * delta_time;

    // 限制角度范围在0-360度
    yaw = fmodf(yaw, 360.0f);
    if (yaw < 0.0f) yaw += 360.0f;
}

/**
  * @brief  发送IMU数据到蓝牙（偏航角作为首位）
  * @retval None
  */
void Send_IMU_Data_Bluetooth(void) {
    char bluetooth_buf[256];

    // 温度转换
    float temperature = (temp / 340.0f) + 36.53f;

    // JSON格式输出，angles数组顺序为[yaw, roll, pitch]
    int len = snprintf(bluetooth_buf, sizeof(bluetooth_buf),
        "{\"angles\":[%.1f,%.1f,%.1f],\"accel\":[%d,%d,%d],\"gyro\":[%d,%d,%d],\"temp\":%.1f}\r\n",
        yaw, roll, pitch,  // 偏航角作为首位
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z,
        temperature);

    // 通过蓝牙发送
    HAL_UART_Transmit(&huart1, (uint8_t*)bluetooth_buf, len, 100);
}

/**
  * @brief  应用程序入口
  * @retval int
  */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_I2C1_Init();

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);

    // 初始化MPU6500
    MPU6500_Init(&hi2c1);
    MPU6500_Calibrate(&hi2c1);

    // 初始化零偏校准相关变量
    gyro_z_bias = 0.0f;
    is_stationary = 0;
    stationary_start_time = 0;

    HAL_Delay(50);
    uint8_t startCmd[] = { 0xA5, 0x20 };
    HAL_UART_Transmit(&huart6, startCmd, sizeof(startCmd), 100);
    HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);

    // 初始化编码器值
    lastEncoderA = __HAL_TIM_GET_COUNTER(&htim2);
    lastEncoderB = __HAL_TIM_GET_COUNTER(&htim4);

    // 初始化PID控制器
    pid_left.Kp = PID_KP;
    pid_left.Ki = PID_KI;
    pid_left.Kd = PID_KD;
    pid_left.integral = 0.0f;
    pid_left.prev_error = 0.0f;
    pid_left.integral_limit = PID_INTEGRAL_LIMIT;

    pid_right.Kp = PID_KP;
    pid_right.Ki = PID_KI;
    pid_right.Kd = PID_KD;
    pid_right.integral = 0.0f;
    pid_right.prev_error = 0.0f;
    pid_right.integral_limit = PID_INTEGRAL_LIMIT;

    uint32_t last_imu_time = HAL_GetTick();
    uint32_t last_debug_time = HAL_GetTick();
    uint32_t last_battery_time = HAL_GetTick();

    while (1) {
        SendConnectionNotification();
        UpdateSpeedRamp();

        // 处理自动转向
        if (auto_turn_mode != 0) {
            HandleAutoTurn();
        }

        // 非自动转向模式下的正常控制
        if (auto_turn_mode == 0) {
            // 读取编码器值并计算转速
            int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
            int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);
            int32_t deltaA = encoderA - lastEncoderA;
            int32_t deltaB = encoderB - lastEncoderB;
            lastEncoderA = encoderA;
            lastEncoderB = encoderB;

            // 计算实际转速 (RPM)
            float rpmA = (float)deltaA / PPR * (60000.0f / SAMPLE_TIME_MS);
            float rpmB = (float)deltaB / PPR * (60000.0f / SAMPLE_TIME_MS);

            // 根据当前方向设置目标转速
            if (current_direction == DIR_FORWARD) {
                target_rpm = current_speed * RPM_TO_PWM_FACTOR;
            }
            else if (current_direction == DIR_BACKWARD) {
                target_rpm = -current_speed * RPM_TO_PWM_FACTOR;
            }
            else {
                target_rpm = 0;
            }

            // 在前进/后退时使用PID控制
            if (current_direction == DIR_FORWARD || current_direction == DIR_BACKWARD) {
                float pid_output_left = PID_Update(&pid_left, target_rpm, rpmA);
                float pid_output_right = PID_Update(&pid_right, target_rpm, rpmB);

                pwm_left = fmaxf(0, fminf(current_speed + pid_output_left, MAX_SPEED));
                pwm_right = fmaxf(0, fminf(current_speed + pid_output_right, MAX_SPEED_RIGHT));
            }
            else {
                pwm_left = current_speed;
                pwm_right = current_speed;
            }

            // 控制电机
            ControlMotor(current_direction, current_speed);
            
            // 每500ms输出一次PID调试信息
            if (HAL_GetTick() - last_debug_time > 100) {
            last_debug_time = HAL_GetTick();
            snprintf(uart_buf, sizeof(uart_buf),
                "T:%.1f, A:%.1f, B:%.1f, PWM_L:%d, PWM_R:%d\n",
                target_rpm, rpmA, rpmB, (int)pwm_left, (int)pwm_right);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
        }
    }

        // 每100ms读取一次MPU6500数据
        if (HAL_GetTick() - last_imu_time > 100) {
            last_imu_time = HAL_GetTick();

            MPU6500_Read_All(&hi2c1);
            Calculate_Angles();
            Send_IMU_Data_Bluetooth();
        }

        // 每2秒读取一次电池电压
        if (HAL_GetTick() - last_battery_time > 2000) {
            last_battery_time = HAL_GetTick();
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
            float voltage = (adc_val * 3.3f * 11.0f) / 4096.0f;
            snprintf(uart_buf, sizeof(uart_buf), "Battery: %.2fV\n", voltage);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
        }

        // 处理激光雷达数据
        uint32_t currentRxIndex = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        if (currentRxIndex != lidar_rxIndex) {
            lastLidarRxTime = HAL_GetTick();
            uint32_t dataLength;
            if (currentRxIndex > lidar_rxIndex) {
                dataLength = currentRxIndex - lidar_rxIndex;
                for (uint32_t i = 0; i < dataLength; i++) {
                    uint8_t data = lidar_dma_buffer[lidar_rxIndex + i];
                    if (lidar_dataIndex == 0 && data != 0xA5) continue;
                    lidar_dataBuffer[lidar_dataIndex++] = data;
                    if (lidar_dataIndex >= DATA_PACKET_SIZE) {
                        ProcessLidarData(lidar_dataBuffer);
                        lidar_dataIndex = 0;
                    }
                }
            }
            else {
                dataLength = DMA_BUFFER_SIZE - lidar_rxIndex;
                for (uint32_t i = 0; i < dataLength; i++) {
                    uint8_t data = lidar_dma_buffer[lidar_rxIndex + i];
                    if (lidar_dataIndex == 0 && data != 0xA5) continue;
                    lidar_dataBuffer[lidar_dataIndex++] = data;
                    if (lidar_dataIndex >= DATA_PACKET_SIZE) {
                        ProcessLidarData(lidar_dataBuffer);
                        lidar_dataIndex = 0;
                    }
                }
                if (currentRxIndex > 0) {
                    for (uint32_t i = 0; i < currentRxIndex; i++) {
                        uint8_t data = lidar_dma_buffer[i];
                        if (lidar_dataIndex == 0 && data != 0xA5) continue;
                        lidar_dataBuffer[lidar_dataIndex++] = data;
                        if (lidar_dataIndex >= DATA_PACKET_SIZE) {
                            ProcessLidarData(lidar_dataBuffer);
                            lidar_dataIndex = 0;
                        }
                    }
                }
            }
            lidar_rxIndex = currentRxIndex;
        }
        if (lidar_dataIndex > 0 && (HAL_GetTick() - lastLidarRxTime > LIDAR_TIMEOUT_THRESHOLD)) {
            lidar_dataIndex = 0;
        }

        HAL_Delay(10);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */