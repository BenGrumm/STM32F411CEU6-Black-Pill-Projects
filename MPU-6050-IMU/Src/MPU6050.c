#include "MPU6050.h"

/**
 * @brief MPU setup function that will initialise MPU struct and setup the MPU6050 for use
 * 
 * @param mpu struct to store data
 * @param i2c_handler handler of the I2C bus to use
 * @return uint8_t 0 if no errors else the amount of errors / timeouts / busy
 */
uint8_t setupMPU6050(MPU6050* mpu, I2C_HandleTypeDef* i2c_handler){
    int returnError = 0;

    // Initialise the mpu struct
    mpu->i2c_handler = i2c_handler;

    for(int i = 0; i < 3; i++){
        mpu->accelValues[i] = 0;
        mpu->gyroValues[i] = 0;
    }

    mpu->lastGyroReadingTime = HAL_GetTick();

    // MPU-6050 addr = 0x68
    uint8_t tempReg = 0;

    // Configure power settings
    tempReg = 0;
    HAL_StatusTypeDef error = MPU6050_writeRegisters(mpu, MPU_PWR_MGMT_1, &tempReg, 1);

    returnError += (error != HAL_OK);

    // Configure gyro FS_SEL for scale range
    tempReg = 0x10;
    error = MPU6050_writeRegisters(mpu, MPU_GYRO_CONFIG, &tempReg, 1);

    returnError += (error != HAL_OK);

    // Configure Accell settings
    tempReg = 0;
    error = MPU6050_writeRegisters(mpu, MPU_ACCEL_CONFIG, &tempReg, 1);

    returnError += (error != HAL_OK);

    return returnError;
}

/**
 * @brief Function to convert 8 bit unsigned int values retrived from slave into the intended signed 16 bit values
 * 
 * @param values values that need to be converted 
 * @param output where to store the converted values
 * @param len the length of the input values array and double the length of the output array
 */
void convertUnsigned8ToSigned16(uint8_t* values, int16_t* output, uint8_t len){
    for(int i = 0; i < len; i+=2){
        output[i / 2] = ((int16_t) values[i]) << 8;
        output[i / 2] |= values[i + 1];
    }
}

HAL_StatusTypeDef MPU6050_readAndConvert6Regs(MPU6050* device, uint8_t addr, int16_t* output){
    uint8_t regValues[6];
    HAL_StatusTypeDef returnVal = MPU6050_readRegisters(device, addr, regValues, 6);

    if(returnVal == HAL_OK){
        convertUnsigned8ToSigned16(regValues, output, 6);
    }
    
    return returnVal;
}

HAL_StatusTypeDef MPU6050_readGyro(MPU6050* device){    
    // Read from MPU_GYRO_XOUT_H - MPU_GYRO_ZOUT_L (6 8 bit registers) with high / low regs for HYZ
    return MPU6050_readAndConvert6Regs(device, MPU_GYRO_XOUT_H, device->gyroValues);
}

HAL_StatusTypeDef MPU6050_readAccelerometer(MPU6050* device){
    // Read from MPU_ACCEL_XOUT_H - MPU_ACCEL_ZOUT_L (6 8 bit registers) with high / low regs for X,Y,Z
    return MPU6050_readAndConvert6Regs(device, MPU_ACCEL_XOUT_H, device->gyroValues);
}

HAL_StatusTypeDef MPU6050_readRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen){
    return HAL_I2C_Mem_Read(device->i2c_handler, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, dataLen, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_writeRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen){
    return HAL_I2C_Mem_Write(device->i2c_handler, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, dataLen, HAL_MAX_DELAY);
}