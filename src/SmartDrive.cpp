/*
 * The MIT License (MIT)
 *
 * Author: Oussema Harbi <oussema.elharbi@gmail.com>
 * Copyright (c) <2016> <Oussema Harbi>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * Ported by Elton Vieira elton.a.vieira@gmail.com from original writed to linux SO 
 * to arduino and esp32 architectures
 */

#include "SmartDrive.h"
#include <Wire.h>

SmartDrive::SmartDrive(byte address):_address(address)
{
    Wire.begin(); 
}

void SmartDrive::writeByte(byte reg, byte value) {
    Wire.beginTransmission(this->_address);  // select device
    Wire.write(reg);                        // select base register
    Wire.write(value);                      // write value
    Wire.endTransmission();                 // stop transmitting
}

byte SmartDrive::readByte(byte reg) {
    Wire.beginTransmission(this->_address);      // select device
    Wire.write(reg);                            // select base register
    Wire.endTransmission(false);                // resend begin
    Wire.requestFrom(this->_address, 1, true);    // request byte    
    while(Wire.available() == false);           // wait for byte arrive   
    return Wire.read();
}

void SmartDrive::writeArray(byte* array, byte len) {
    Wire.beginTransmission(this->_address);  // select device
    Wire.write(array, len);                 // select base register and writing values
    Wire.endTransmission();                 // stop transmitting
}

unsigned int SmartDrive::readInteger(byte reg) {
    unsigned int result;
    Wire.beginTransmission(this->_address);      // select device
    Wire.write(reg);                            // select base register
    Wire.endTransmission(false);                // resend begin
    Wire.requestFrom(this->_address, 2, true);    // request byte    
    while(Wire.available() < 2);                // wait for bytes arrive   
    result =  Wire.read();
    result = (Wire.read() << 8) | result;
    return result;
}

unsigned long SmartDrive::readLongSigned(byte reg) {
    unsigned long result;

    Wire.beginTransmission(this->_address);      // select device
    Wire.write(reg);                            // select base register
    Wire.endTransmission(false);                // resend begin
    Wire.requestFrom(this->_address, 2, true);    // request byte    
    while(Wire.available() < 4);                // wait for bytes arrive   
    result =  Wire.read();
    result = (Wire.read() << 8)  | result;
    result = (Wire.read() << 16) | result;
    result = (Wire.read() << 24) | result;
    return result;
}

void SmartDrive::InternalDelay() {
    delay(1000);
}

void SmartDrive::command(byte cmd) {
    // std::cout << "Running Command : " << cmd << std::endl;
    writeByte(SmartDrive_COMMAND, cmd);
}


float SmartDrive::GetBattVoltage() {
    byte value = 0;

    value = readByte(SmartDrive_BATT_VOLTAGE);
    return (value * SmartDrive_VOLTAGE_MULTIPLIER);
}

unsigned long SmartDrive::ReadTachometerPosition(int motor_id) {
    if (motor_id == 1 )
        return readLongSigned(SmartDrive_POSITION_M1);
    else
        return readLongSigned(SmartDrive_POSITION_M2);
}

void SmartDrive::Run_Unlimited(int motor_id, int direction, byte speed) {
    byte ctrl = 0;
    ctrl |= SmartDrive_CONTROL_SPEED;
    ctrl |= SmartDrive_CONTROL_BRK;

    // std::cout << "Running with speed : " << (int) speed << std::endl;

    if ( motor_id != SmartDrive_Motor_ID_BOTH )
        ctrl |= SmartDrive_CONTROL_GO;
    if ( direction != SmartDrive_Dir_Forward )
        speed = speed * -1;
    if ( motor_id != SmartDrive_Motor_ID_2) {
        byte array [5] = {SmartDrive_SPEED_M1, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id != SmartDrive_Motor_ID_1) {
        byte array [5] = {SmartDrive_SPEED_M2, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id == SmartDrive_Motor_ID_BOTH )
        writeByte(SmartDrive_COMMAND, CMD_S);
}

void SmartDrive::StopMotor(int motor_id, int next_action ) {
    if ( next_action !=SmartDrive_Action_Float )
        writeByte(SmartDrive_COMMAND, CMD_A+motor_id-1);
    else
        writeByte(SmartDrive_COMMAND, CMD_a+motor_id-1);
}

void SmartDrive::Run_Seconds(int motor_id, int direction, byte speed, byte duration, bool wait_for_completion, int next_action ) {
    byte ctrl = 0;
    ctrl |= SmartDrive_CONTROL_SPEED;
    ctrl |= SmartDrive_CONTROL_TIME;

    if ( next_action ==SmartDrive_Action_Brake )
        ctrl |= SmartDrive_CONTROL_BRK;
    if ( next_action ==SmartDrive_Action_BrakeHold ) {
        ctrl |= SmartDrive_CONTROL_BRK;
        ctrl |= SmartDrive_CONTROL_ON;
    }
    if ( motor_id != SmartDrive_Motor_ID_BOTH )
        ctrl |= SmartDrive_CONTROL_GO;
    if ( direction != SmartDrive_Dir_Forward )
        speed = speed * -1;
    if ( motor_id != SmartDrive_Motor_ID_2) {
        byte array[5] = {SmartDrive_SPEED_M1, speed, duration, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id != SmartDrive_Motor_ID_1) {
        byte array[5] = {SmartDrive_SPEED_M2, speed, duration, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id == SmartDrive_Motor_ID_BOTH )
        writeByte(SmartDrive_COMMAND, CMD_S);
    if ( wait_for_completion ) {
        InternalDelay(); //this delay is required for the status byte to be available for reading.
        WaitUntilTimeDone(motor_id);
    }
}

void SmartDrive::WaitUntilTimeDone(int motor_id) {
        while (IsTimeDone(motor_id) == false)
            InternalDelay();
}

bool SmartDrive::IsTimeDone(int motor_id) {
    byte result_1 = 0, result_2 = 0;
    if ( motor_id != SmartDrive_Motor_ID_2 )
        result_1 = readByte(SmartDrive_STATUS_M1);
    if ( motor_id != SmartDrive_Motor_ID_1 )
        result_2 = readByte(SmartDrive_STATUS_M2);
    return (((result_1 & 0x40) == 0) && ((result_2 & 0x40) == 0) );  //look for time bits to be zero
}

void SmartDrive::Run_Degrees(int motor_id, int direction, byte speed, unsigned long degrees, bool wait_for_completion, int next_action) {
    byte ctrl = 0;
    ctrl |= SmartDrive_CONTROL_SPEED;
    ctrl |= SmartDrive_CONTROL_TACHO;
    ctrl |= SmartDrive_CONTROL_RELATIVE;

    unsigned long d = degrees;
    if ( direction != SmartDrive_Dir_Forward )
        d = degrees * -1 ;

    byte t4 = (d/0x1000000);
    byte t3 = ((d%0x1000000)/0x10000);
    byte t2 = (((d%0x1000000)%0x10000)/0x100);
    byte t1 = (((d%0x1000000)%0x10000)%0x100);

    if ( next_action ==SmartDrive_Action_Brake )
        ctrl |= SmartDrive_CONTROL_BRK;
    if ( next_action ==SmartDrive_Action_BrakeHold ) {
        ctrl |= SmartDrive_CONTROL_BRK;
        ctrl |= SmartDrive_CONTROL_ON;
    }
    if ( motor_id != SmartDrive_Motor_ID_BOTH )
        ctrl |= SmartDrive_CONTROL_GO;
    if ( motor_id != SmartDrive_Motor_ID_2) {
        byte array[9] = {SmartDrive_SETPT_M1, t1, t2, t3, t4, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id != SmartDrive_Motor_ID_1){
        byte array[9] = {SmartDrive_SETPT_M2, t1, t2, t3, t4, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id == SmartDrive_Motor_ID_BOTH )
        writeByte(SmartDrive_COMMAND, CMD_S);
    if ( wait_for_completion ) {
        InternalDelay();//this delay is required for the status byte to be available for reading.
        WaitUntilTachoDone(motor_id);
    }
}

void SmartDrive::Run_Rotations(int motor_id, int direction, byte speed, unsigned long rotations, bool wait_for_completion, int next_action) {
    byte ctrl = 0;
    ctrl |= SmartDrive_CONTROL_SPEED;
    ctrl |= SmartDrive_CONTROL_TACHO;
    ctrl |= SmartDrive_CONTROL_RELATIVE;

    unsigned long d = rotations * 360;
    if ( direction != SmartDrive_Dir_Forward )
        d = (rotations * 360) * -1;

    byte t4 = (d/0x1000000);
    byte t3 = ((d%0x1000000)/0x10000);
    byte t2 = (((d%0x1000000)%0x10000)/0x100);
    byte t1 = (((d%0x1000000)%0x10000)%0x100);

    if ( next_action ==SmartDrive_Action_Brake )
        ctrl |= SmartDrive_CONTROL_BRK;
    if ( next_action ==SmartDrive_Action_BrakeHold ) {
        ctrl |= SmartDrive_CONTROL_BRK;
        ctrl |= SmartDrive_CONTROL_ON;
    }
    if ( motor_id != SmartDrive_Motor_ID_BOTH )
        ctrl |= SmartDrive_CONTROL_GO;
    if ( motor_id != SmartDrive_Motor_ID_2) {
        byte array[9] = {SmartDrive_SETPT_M1, t1, t2, t3, t4, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id != SmartDrive_Motor_ID_1) {
        byte array[9] = {SmartDrive_SETPT_M2, t1, t2, t3, t4, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id == SmartDrive_Motor_ID_BOTH )
        writeByte(SmartDrive_COMMAND, CMD_S);
    if ( wait_for_completion) {
        InternalDelay(); //this delay is required for the status byte to be available for reading.
        WaitUntilTachoDone(motor_id);
    }
}

void SmartDrive::Run_Tacho(int motor_id, byte speed, unsigned long tacho_count, bool wait_for_completion, int next_action) {
    byte ctrl = 0;
    ctrl |= SmartDrive_CONTROL_SPEED;
    ctrl |= SmartDrive_CONTROL_TACHO;

    unsigned long d = tacho_count;

    byte t4 = (d/0x1000000);
    byte t3 = ((d%0x1000000)/0x10000);
    byte t2 = (((d%0x1000000)%0x10000)/0x100);
    byte t1 = (((d%0x1000000)%0x10000)%0x100);

    if ( next_action ==SmartDrive_Action_Brake )
        ctrl |= SmartDrive_CONTROL_BRK;
    if ( next_action ==SmartDrive_Action_BrakeHold ) {
        ctrl |= SmartDrive_CONTROL_BRK;
        ctrl |= SmartDrive_CONTROL_ON;
    }
    if ( motor_id != SmartDrive_Motor_ID_BOTH )
        ctrl |= SmartDrive_CONTROL_GO;
    if ( motor_id != SmartDrive_Motor_ID_2){
        byte array[9]= {SmartDrive_SETPT_M1, t1, t2, t3, t4, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id != SmartDrive_Motor_ID_1){
        byte array[9]= {SmartDrive_SETPT_M2, t1, t2, t3, t4, speed, 0, 0, ctrl};
        writeArray(array, sizeof(array));
    }
    if ( motor_id == SmartDrive_Motor_ID_BOTH )
        writeByte(SmartDrive_COMMAND, CMD_S);
    if ( wait_for_completion )
        InternalDelay(); //this delay is required for the status byte to be available for reading.
        WaitUntilTachoDone(motor_id);
}

void SmartDrive::WaitUntilTachoDone(int motor_id) {
    while (IsTachoDone(motor_id) == false)
        InternalDelay();
}

bool SmartDrive::IsTachoDone(int motor_id) {
    byte result_1 = 0, result_2 = 0;

    if ( motor_id != SmartDrive_Motor_ID_2 )
        result_1 = readByte(SmartDrive_STATUS_M1);
    if ( motor_id != SmartDrive_Motor_ID_1 )
        result_2 = readByte(SmartDrive_STATUS_M2);
    //look for both time bits to be zero
    return (((result_1 & 0x08) == 0) && ((result_2 & 0x08) == 0) );
}

void SmartDrive::SetPerformanceParameters( unsigned int Kp_tacho, unsigned int Ki_tacho, unsigned int Kd_tacho, unsigned int Kp_speed, unsigned int Ki_speed, unsigned int Kd_speed, byte passcount, byte tolerance) {
    byte Kp_t1 = Kp_tacho%0x100;
    byte Kp_t2 = Kp_tacho/0x100;
    byte Ki_t1 = Ki_tacho%0x100;
    byte Ki_t2 = Ki_tacho/0x100;
    byte Kd_t1 = Kd_tacho%0x100;
    byte Kd_t2 = Kd_tacho/0x100;
    byte Kp_s1 = Kp_speed%0x100;
    byte Kp_s2 = Kp_speed/0x100;
    byte Ki_s1 = Ki_speed%0x100;
    byte Ki_s2 = Ki_speed/0x100;
    byte Kd_s1 = Kd_speed%0x100;
    byte Kd_s2 = Kd_speed/0x100;

    byte array[15] = {SmartDrive_P_Kp, Kp_t1 , Kp_t2 , Ki_t1, Ki_t2, Kd_t1, Kd_t2, Kp_s1, Kp_s2, Ki_s1, Ki_s2, Kd_s1, Kd_s2, passcount, tolerance};
    writeArray(array, sizeof(array));
}

unsigned int SmartDrive::ReadKpTacho()
{
    return readInteger(SmartDrive_P_Kp);
}

unsigned int SmartDrive::ReadKiTacho()
{
    return readInteger(SmartDrive_P_Ki); 
}

unsigned int SmartDrive::ReadKdTacho()
{
    return readInteger(SmartDrive_P_Kd); 
}

unsigned int SmartDrive::ReadKpSpeed()
{
    return readInteger(SmartDrive_S_Kp);
}

unsigned int SmartDrive::ReadKiSpeed()
{
    return readInteger(SmartDrive_S_Ki);
}

unsigned int SmartDrive::ReadKdSpeed()
{
    return readInteger(SmartDrive_S_Kd);
}

unsigned int SmartDrive::ReadPasscount()
{
    return readInteger(SmartDrive_PASSCOUNT);
}

unsigned int SmartDrive::ReadTolerance()
{
    return readInteger(SmartDrive_PASSTOLERANCE);
}

byte SmartDrive::GetMotorStatus(int motor_id) {
    byte status=0;
    if (motor_id == SmartDrive_Motor_ID_1)
        status = readByte(SmartDrive_STATUS_M1);
    if (motor_id == SmartDrive_Motor_ID_2)
        status = readByte(SmartDrive_STATUS_M1);
    if (motor_id == SmartDrive_Motor_ID_BOTH) {
        // std::cout << "Please specify which motor's status you want to fetch !" << std::endl;
    }
    return status;
}

bool SmartDrive::IsMotorSpeedFixed(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_CONTROL_ON)
        result = true;

    return result;
}

bool SmartDrive::IsMotorRamping(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_IS_RAMPING)
        result = true;

    return result;
}

bool SmartDrive::IsMotorPowered(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_IS_POWERED)
        result = true;

    return result;
}

bool SmartDrive::IsMotorMovingTowardEncoder(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_POS_CTRL_ON)
        result = true;

    return result;
}

bool SmartDrive::IsMotorInBrakeMode(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_IN_BRAKE_MODE)
        result = true;

    return result; 
}

bool SmartDrive::IsMotorOverloaded(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_OVERLOADED)
        result = true;

    return result; 
}

bool SmartDrive::IsMotorInTimeMode(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_IN_TIME_MODE)
        result = true;

    return result; 
}

bool SmartDrive::IsMotorStalled(int motor_id)
{
    bool result = false;
    byte status = GetMotorStatus(motor_id);

    if (status & SmartDrive_MOTOR_IS_STALLED)
        result = true;

    return result; 
}
