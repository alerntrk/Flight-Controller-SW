
/*
 * pid.c
 *
 *  Created on: Mar 14, 2023
 *      Author: AEren TURK
 */

#include "mpu.h"
#include "sensorDrive.h"
#include <math.h>

static uint32_t prev_time,time_,elapsedTime;
extern float dt;
extern unsigned long t1;
extern unsigned long t2;


extern float meanR;
extern float meanP;
float sum=0;
float summ=0;
int i=0;

static sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange);
static sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange);

sensor_status_e MPU6050_initialize( SensorData_t *pSensor, fs_sel_e gyroConfig, afs_sel_e acc_config )
{
    sensor_status_e retVal;

    retVal = MPU6050_set_gyro_range( pSensor, gyroConfig );

    if (retVal == SENSOR_OK) {
        retVal = MPU6050_set_acc_range( pSensor, acc_config );

        if (retVal == SENSOR_OK) {
            retVal = MPU6050_set_sleep_mode( SLEEPMODE_OFF );
        }
    }


    return retVal;
}

sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange)
{
	sensor_status_e retVal;
	uint8_t configReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_GYRO_CONFIG );

	configReg |= ((uint32_t)gyroRange << MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION );

	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_GYRO_CONFIG, configReg);

	switch (gyroRange)
	{
	 case FS_250:  pSensor->gyro_co = 131.0; break;
	 case FS_500:  pSensor->gyro_co = 65.5; break;
	 case FS_1000: pSensor->gyro_co = 32.8; break;
	 case FS_2000: pSensor->gyro_co = 16.4; break;
	 default: retVal = SENSOR_ERROR; break;
	}

	return retVal;
}

sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange)
{
	sensor_status_e retVal;
	uint8_t configReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_ACCEL_CONFIG );

	configReg |= ( (uint32_t) accRange << MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION );

	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_ACCEL_CONFIG, configReg);

	switch (accRange)
	{
	 case AFS_2G:  pSensor->acc_co = 16384; break;
	 case AFS_4G:  pSensor->acc_co = 8192; break;
	 case AFS_8G:  pSensor->acc_co = 4096; break;
	 case AFS_16G: pSensor->acc_co = 2048; break;
	 default: retVal = SENSOR_ERROR; break;
	}

	return retVal;
}

sensor_status_e MPU6050_test_sensor()
{
	sensor_status_e retVal;
	retVal = sensor_test_device(MPU6050_I2C_ADRESS_AD1);
	return retVal;
}


uint8_t MPU6050_read_id(void)
{
	uint8_t id = 0;
	id = sensor_read_register8(MPU6050_I2C_ADRESS_AD1,MPU_REG_WHO_AM_I );
	return id;
}

sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleepmode)
{
	sensor_status_e retVal;
	uint8_t powerReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_PWR_MGMT_1 );

	if(SLEEPMODE_ON == sleepmode) {
		SET_BIT(powerReg,1<<MPU_BIT_PWR_MGMT_1_SLEEP_MODE);
	}
	else {
		CLEAR_BIT(powerReg,1<<MPU_BIT_PWR_MGMT_1_SLEEP_MODE);
	}
	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_PWR_MGMT_1, powerReg);

	powerReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_PWR_MGMT_1 );

	return retVal;
}


sensor_status_e MPU6050_read_data(SensorData_t *pSensorData)
{
	sensor_status_e retVal;
	uint8_t buffer[14];
	retVal =  sensor_read_bytes(MPU6050_I2C_ADRESS_AD1, MPU_REG_ACCEL_XOUT_H, buffer, 14);

    if (retVal == SENSOR_OK) {
        pSensorData->accRaw.X = (int16_t) ((buffer[0] << 8) | buffer[1]);
        pSensorData->accRaw.Y = (int16_t) ((buffer[2] << 8) | buffer[3]);
        pSensorData->accRaw.Z = (int16_t) ((buffer[4] << 8) | buffer[5]);

        pSensorData->gyroRaw.X = (int16_t) ((buffer[8] << 8) | buffer[9]);
        pSensorData->gyroRaw.Y = (int16_t) ((buffer[10] << 8) | buffer[11]);
        pSensorData->gyroRaw.Z = (int16_t) ((buffer[12] << 8) | buffer[13]);

        pSensorData->acc.X = (float)(pSensorData->accRaw.X - pSensorData->acc.xOffSet) / pSensorData->acc_co;
        pSensorData->acc.Y = (float)(pSensorData->accRaw.Y - pSensorData->acc.yOffSet) / pSensorData->acc_co;
        pSensorData->acc.Z = (float)(pSensorData->accRaw.Z - pSensorData->acc.zOffSet) / pSensorData->acc_co;

        pSensorData->gyro.X = (float)(pSensorData->gyroRaw.X - pSensorData->gyro.xOffSet) / pSensorData->gyro_co;
        pSensorData->gyro.Y = (float)(pSensorData->gyroRaw.Y - pSensorData->gyro.yOffSet) / pSensorData->gyro_co;
        pSensorData->gyro.Z = (float)(pSensorData->gyroRaw.Z - pSensorData->gyro.zOffSet) / pSensorData->gyro_co;

        return retVal;
    } else {
        return SENSOR_ERROR;
    }
}

void CalcOffSet(SensorData_t *sensorData){

		sensorData->gyro.xOffSet=0;
		sensorData->gyro.yOffSet=0;
		sensorData->gyro.zOffSet=0;



		sensorData->acc.xOffSet=0;
		sensorData->acc.yOffSet=0;
		sensorData->acc.zOffSet=0;


	 float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro

	 for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
	    MPU6050_read_data(sensorData);
		ag[0] += sensorData->acc.X;
		ag[1] += sensorData->acc.Y;
		ag[2] += (sensorData->acc.Z-1.0);
		ag[3] += sensorData->gyro.X;
		ag[4] += sensorData->gyro.Y;
		ag[5] += sensorData->gyro.Z;
		HAL_Delay(1); // wait a little bit between 2 measurements
	  }

		sensorData->acc.xOffSet = ag[0] / CALIB_OFFSET_NB_MES;
		sensorData->acc.yOffSet= ag[1] / CALIB_OFFSET_NB_MES;
		sensorData->acc.zOffSet = ag[2] / CALIB_OFFSET_NB_MES;



		sensorData->gyro.xOffSet = ag[0] / CALIB_OFFSET_NB_MES;
		sensorData->gyro.yOffSet= ag[1] / CALIB_OFFSET_NB_MES;
		sensorData->gyro.zOffSet = ag[2] / CALIB_OFFSET_NB_MES;


}
/*
void applyMovingAverageFilter(float *dataP,float* dataR, int size) {


    // Apply the filter to each data point
    for (int i = 0; i < size; i++) {
        // Calculate the sum of the data points within the window
        float sum = 0.0;
        float summ=0;
        int windowStart = i - (WINDOW_SIZE / 2);
        int windowEnd = i + (WINDOW_SIZE / 2);

        if (windowStart < 0) {
            windowStart = 0;
        }
        if (windowEnd >= size) {
            windowEnd = size - 1;
        }

        for (int j = windowStart; j <= windowEnd; j++) {
            sum += dataP[j];
            summ += dataR[j];
        }

        // Calculate the average value
        filteredPitch[i] = sum / (windowEnd - windowStart + 1);
        filteredRoll[i] = summ / (windowEnd - windowStart + 1);
    }
    int k=0;
    float sum2=0;
    float sum3=0;
    while(k<size){
    	sum2+=filteredPitch[k];
    	sum3+=filteredRoll[k];
    	k++;
    }
    meanP=(float)sum2/(float)size;
    meanR=(float)sum3/(float)size;

}
*/
void mpu_Update(SensorData_t* pSensorData) {

	/*prev_time=time;
	time = HAL_GetTick();
	elapsedtime=(time-prevtime)*1000;

	MPU6050_read_data(pSensorData);
	pSensorData->gyro.pitch += (pSensorData->gyroRaw.X - pSensorData->gyro.xOffSet)*0.004;
	pSensorData->gyro.roll += (pSensorData->gyroRaw.Y - pSensorData->gyro.yOffSet)*0.004;

	acc_total_vec=sqrt(pSensorData->accRaw.X*pSensorData->accRaw.X + pSensorData->accRaw.Y*pSensorData->accRaw.Y+pSensorData->accRaw.Z*pSensorData->accRaw.Z);
	pSensorData->acc.pitch =asin(pSensorData->accRaw.Y/acc_total_vec);
	pSensorData->acc.roll =asin(pSensorData->accRaw.X/acc_total_vec);

	pSensorData->AnglePitch= pSensorData->AnglePitch*0.9 +pSensorData->acc.pitch*0.1;
	pSensorData->AngleRoll= pSensorData->AngleRoll*0.9 +pSensorData->acc.roll*0.1;
	while((HAL_GetTick() - prevtime)*1000 < 4000);
	prevtime = HAL_GetTick();*/

	t1 = DWT->CYCCNT;



	double acc_total_vec;


	MPU6050_read_data(pSensorData);

	pSensorData->gyro.pitch += pSensorData->gyro.X*0.004;
	pSensorData->gyro.roll += pSensorData->gyro.Y*0.004;

	pSensorData->gyro.pitch += pSensorData->gyro.roll * sin(pSensorData->gyro.Z*0.000001066);
	pSensorData->gyro.roll += pSensorData->gyro.pitch * sin(pSensorData->gyro.Z*0.000001066);

	acc_total_vec=sqrt(pSensorData->acc.X*pSensorData->acc.X + pSensorData->acc.Y*pSensorData->acc.Y+pSensorData->acc.Z*pSensorData->acc.Z);

	pSensorData->acc.pitch = asin(pSensorData->acc.Y/acc_total_vec)*57.296;
	pSensorData->acc.roll = asin(pSensorData->acc.X/acc_total_vec)*57.296;

	pSensorData->AnglePitch =pSensorData->gyro.pitch*0.0004 + pSensorData->acc.pitch*0.9996+1.8;
	pSensorData->AngleRoll =pSensorData->gyro.roll*0.0004 + pSensorData->acc.roll*0.9996+0.3;
	//sensorData->angleZ += sensorData->gyro.Z*dt;

	t2 = DWT->CYCCNT;
	dt = (double)(t2 - t1);
	pSensorData->AngleYaw = (double)pSensorData->gyro.Z*dt/(double)1000000;
	while((HAL_GetTick() - prev_time)*1000 < 4000);

	prev_time = HAL_GetTick();


	int j=0;
	if(i<200){
		sum+=pSensorData->AnglePitch;
		i++;
		summ+=pSensorData->AngleRoll;

	}

	if(i>=200)
		{
		meanP=(float)sum/(float)200-0.35;
		meanR=(float)summ/(float)200+4.7;
			i=0;
			sum=0;
			summ=0;

		}

	/*while(i<20){

		sum+=pSensorData->AnglePitch
		//data_Pitch[i]=pSensorData->AnglePitch;
		//data_Roll[i]=pSensorData->AngleRoll;
		i++;


	}
	meanP=sum/50;
	if(i==50){

		i=0;

	}*/

	//applyMovingAverageFilter(data_Pitch,data_Roll,50);



}
// Function to apply the moving average filter to noisy data


