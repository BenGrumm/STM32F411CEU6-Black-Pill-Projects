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
        mpu->accel_angle[i] = 0;
        mpu->gyro_angle[i] = 0;
        mpu->position[i] = 0;
    }

    mpu->lastGyroReadingTime = HAL_GetTick();

    // MPU-6050 addr = 0x68
    uint8_t tempReg = 0;

    // Configure power settings
    tempReg = 0;
    HAL_StatusTypeDef error = MPU6050_writeRegisters(mpu, MPU_PWR_MGMT_1, &tempReg, 1);

    returnError += (error != HAL_OK);

    // Configure gyro FS_SEL for scale range
    tempReg = 0 | (mpu->MPU_Gyro_Range << 3);
    error = MPU6050_writeRegisters(mpu, MPU_GYRO_CONFIG, &tempReg, 1);

    returnError += (error != HAL_OK);

    // Configure Accell settings
    tempReg = 0 | (mpu->MPU_Accel_Range << 3);
    error = MPU6050_writeRegisters(mpu, MPU_ACCEL_CONFIG, &tempReg, 1);

    returnError += (error != HAL_OK);

    return returnError;
}

HAL_StatusTypeDef MPU6050_readMPUAndCalculatePosition(MPU6050* device){
    float gyroTemp[3], accelTemp[3];
    uint32_t previouseGyroTime = device->lastGyroReadingTime;
    device->lastGyroReadingTime = HAL_GetTick();

    float elapsedTime = (device->lastGyroReadingTime - previouseGyroTime) / 1000.0; // Calculate in seconds

    HAL_StatusTypeDef error = MPU6050_readGyro(device, gyroTemp);

    if(error == HAL_OK){
        device->gyro_angle[0] = device->gyro_angle[0] + (gyroTemp[0] * elapsedTime);
        device->gyro_angle[1] = device->gyro_angle[1] + (gyroTemp[1] * elapsedTime);
        device->gyro_angle[2] = device->gyro_angle[2] + (gyroTemp[2] * elapsedTime);
        error = MPU6050_readAccelerometer(device, accelTemp);
    }

    if(error == HAL_OK){
        device->accel_angle[0] = (atan(accelTemp[1] / sqrt(pow(accelTemp[0], 2) + pow(accelTemp[2], 2))) * 180 / M_PI) - 0.58; // Calculate accelerometer X
        device->accel_angle[1] = (atan(-1 * accelTemp[0] / sqrt(pow(accelTemp[1], 2) + pow(accelTemp[2], 2))) * 180 / M_PI) + 1.58; // Calculate accelerometer Y

        device->gyro_angle[0] = 0.96 * device->gyro_angle[0] + 0.04 * device->accel_angle[0];
        device->gyro_angle[1] = 0.96 * device->gyro_angle[1] + 0.04 * device->accel_angle[1];

        device->position[0] = device->gyro_angle[0];
        device->position[1] = device->gyro_angle[1];
        device->position[2] = device->gyro_angle[2];
    }

    return error;
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

HAL_StatusTypeDef MPU6050_readGyro(MPU6050* device, float* values){
    int16_t gyroValues[3];
    // Read from MPU_GYRO_XOUT_H - MPU_GYRO_ZOUT_L (6 8 bit registers) with high / low regs for HYZ
    HAL_StatusTypeDef error = MPU6050_readAndConvert6Regs(device, MPU_GYRO_XOUT_H, gyroValues);

    if(error == HAL_OK){
        for(int i = 0; i < 3; i++){
            switch(device->MPU_Gyro_Range){
                case MPU_GYRO_SCALE_RANGE_250:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_250;
                    break;
                case MPU_GYRO_SCALE_RANGE_500:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_500;
                    break;
                case MPU_GYRO_SCALE_RANGE_1000:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_1000;
                    break;
                case MPU_GYRO_SCALE_RANGE_2000:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_2000;
                    break;
                default:
                    error = HAL_ERROR;
            }
        }
    }

    return error;
}

HAL_StatusTypeDef MPU6050_readAccelerometer(MPU6050* device, float* values){
    int16_t retrivedValues[3];

    // Read from MPU_ACCEL_XOUT_H - MPU_ACCEL_ZOUT_L (6 8 bit registers) with high / low regs for X,Y,Z
    HAL_StatusTypeDef error = MPU6050_readAndConvert6Regs(device, MPU_ACCEL_XOUT_H, retrivedValues);

    if(error == HAL_OK){
        for(int i = 0; i < 3; i++){
            switch(device->MPU_Accel_Range){
                case MPU_ACCEL_SCALE_RANGE_2G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_2G;
                    break;
                case MPU_ACCEL_SCALE_RANGE_4G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_4G;
                    break;
                case MPU_ACCEL_SCALE_RANGE_8G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_8G;
                    break;
                case MPU_ACCEL_SCALE_RANGE_16G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_16G;
                    break;
                default:
                    error = HAL_ERROR;
            }
        }
    }

    return error;
}

HAL_StatusTypeDef MPU6050_readRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen){
    return HAL_I2C_Mem_Read(device->i2c_handler, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, dataLen, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_writeRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen){
    return HAL_I2C_Mem_Write(device->i2c_handler, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, dataLen, HAL_MAX_DELAY);
}