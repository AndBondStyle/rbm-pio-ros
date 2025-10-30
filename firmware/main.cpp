#include <Arduino.h>
#include "Wire.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "SimpleDCMotor.h"
#include "driver.hpp"

#define MON_ENABLE

constexpr float VOLTAGE = 1.0f;
constexpr float VEL_LIM = 10;
const int PWM_FREQ = 50000;

constexpr float VEL_PID_P = 2.0f;
constexpr float VEL_PID_I = 20.0f;
constexpr float VEL_PID_D = 0.001f;
constexpr float VEL_PID_LIM = 400.0f;
constexpr float VEL_LPF_TF = 0.01f;

HardwareSerial UART(PB7, PB6);
Commander commander = Commander(UART);

TwoWire IIC1(PB9, PB8);
DCMotor motor_left = DCMotor();
CustomDriver driver_left = CustomDriver(PA1, PB12);
MagneticSensorI2C sensor_left(MT6701_I2C);
void onMotorLeft(char* cmd){ commander.motor(&motor_left, cmd); }

TwoWire IIC2(PB11, PB10);
DCMotor motor_right = DCMotor();
CustomDriver driver_right = CustomDriver(PC15, PA0);
MagneticSensorI2C sensor_right(MT6701_I2C);
void onMotorRight(char* cmd){ commander.motor(&motor_right, cmd); }

void initMotorStack(char c, DCMotor &motor, CustomDriver &driver, MagneticSensorI2C &sensor, TwoWire &iic) {
    driver.voltage_power_supply = VOLTAGE;
    driver.voltage_limit = VOLTAGE;
    driver.pwm_frequency = PWM_FREQ;
    driver.init();
    sensor.init(&iic);
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);

    motor.voltage_limit = VOLTAGE;
    motor.velocity_limit = VEL_LIM;
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.init();

    motor.PID_velocity.P = VEL_PID_P;
    motor.PID_velocity.I = VEL_PID_I;
    motor.PID_velocity.D = VEL_PID_D;
    motor.PID_velocity.output_ramp = VEL_PID_LIM;
    motor.LPF_velocity.Tf = VEL_LPF_TF;

    #ifdef MON_ENABLE
    motor.useMonitoring(UART);
    motor.monitor_downsample = 100;
    motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_VOLT_Q;
    motor.monitor_start_char = c;
    #endif

    motor.target = 0.0f;
    motor.enable();
}

void setup() {
    UART.begin(115200);
    while (!UART) { delay(10); };
    SimpleFOCDebug::enable(&UART);
    UART.println("init start");

    initMotorStack('L', motor_left, driver_left, sensor_left, IIC1);
    initMotorStack('R', motor_right, driver_right, sensor_right, IIC2);
    commander.add('L', onMotorLeft, "left motor");
    commander.add('R', onMotorRight, "right motor");
}

void loop() {
    motor_left.move();
    motor_right.move();
    commander.run();

    #ifdef MON_ENABLE
    motor_left.monitor();
    motor_right.monitor();
    #endif
}
