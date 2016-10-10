


#include "mraa.hpp"

#include <iostream>
#include <unistd.h>
#include "SFE_LSM9DS0.h"
#include "SparkFun_pca9685_Edison.h"
using namespace std;

#define PERIOD		2000
#define MINUMUM_DC	55
#define AVERAGE_DC	63
#define MAXIMUM_DC	68
#define DELTA		2

static float duty_cycle1 = AVERAGE_DC;
static float duty_cycle2 = AVERAGE_DC;
static float duty_cycle3 = AVERAGE_DC;
static float duty_cycle4 = AVERAGE_DC;

int main() {

	//Declaration of PWM and 9DOF to be used
	LSM9DS0 *imu;
	imu = new LSM9DS0(0x6B, 0x1D);
	uint16_t imuResult = imu->begin();
	cout<<hex<<"Chip ID: 0x"<<imuResult<<dec<<" (should be 0x49d4)"<<endl;


	mraa::I2c* pwm_i2c;
	pwm_i2c = new mraa::I2c(1); // Tell the I2c object which bus it's on.

	pca9685 pwm(pwm_i2c, 0x40); // 0x40 is the default address for the PCA9685.

	pwm.enableServoMode();

	//Here we initialize all the PWMs with the minimum cycle
	//Experimentally we discovered it was 55% duty cycle
	pwm.setChlDuty(0,MINUMUM_DC);
	pwm.setChlDuty(1,MINUMUM_DC);
	pwm.setChlDuty(2,MINUMUM_DC);
	pwm.setChlDuty(3,MINUMUM_DC);

	std::cout << "Turn power on now" << std::endl;

	//Wait 8 seconds so we have time to move physicaly stabilize 9DOF module
	sleep(8);

	//Set each motor with its required duty cycle
	pwm.setChlDuty(0,duty_cycle1);
	pwm.setChlDuty(1,duty_cycle2);
	pwm.setChlDuty(2,duty_cycle3);
	pwm.setChlDuty(3,duty_cycle4);

	std::cout << "Finished sleeping" << std::endl;

	for(;;)
	{
		//Get each reading from 9DOF
		imu->readAccel();
		imu->readMag();
		imu->readGyro();
		imu->readTemp();

//	    cout<<"Accel x: "<<imu->calcAccel(imu->ax)<<" g\t"<<
//	    	"Accel y: "<<imu->calcAccel(imu->ay)<<" g\t"<<
//			"Accel z: "<<imu->calcAccel(imu->az)<<" g\t"<<
//			"Gyro x: "<<imu->calcGyro(imu->gx)<<" deg/s\t"<<
//			"Gyro y: "<<imu->calcGyro(imu->gy)<<" deg/s\t"<<
//			"Gyro z: "<<imu->calcGyro(imu->gz)<<" deg/s"<<endl;

		//Calculate every DC depending on 9DOF accelerometer position
		if(imu->calcAccel(imu->ax) > 0.01)
		{
			if(duty_cycle1 > MINUMUM_DC)
				duty_cycle1 -= DELTA;
			if(duty_cycle2 > MINUMUM_DC)
				duty_cycle2 -= DELTA;
			if(duty_cycle3 < MAXIMUM_DC)
				duty_cycle3 += DELTA;
			if(duty_cycle4 < MAXIMUM_DC)
				duty_cycle4 += DELTA;
//			std::cout << "1" << std::endl;
		}
		else if(imu->calcAccel(imu->ax) < -0.01)
		{
			if(duty_cycle1 < MAXIMUM_DC)
				duty_cycle1 += DELTA;
			if(duty_cycle2 < MAXIMUM_DC)
				duty_cycle2 += DELTA;
			if(duty_cycle3 > MINUMUM_DC)
				duty_cycle3 -= DELTA;
			if(duty_cycle4 > MINUMUM_DC)
				duty_cycle4 -= DELTA;
//	    	std::cout << "2" << std::endl;
		}

		if(imu->calcAccel(imu->ay) > 0.01)
		{
			if(duty_cycle1 > MINUMUM_DC)
				duty_cycle1 -= DELTA;
			if(duty_cycle2 < MAXIMUM_DC)
				duty_cycle2 += DELTA;
			if(duty_cycle3 < MAXIMUM_DC)
				duty_cycle3 += DELTA;
			if(duty_cycle4 > MINUMUM_DC)
				duty_cycle4 -= DELTA;
//	    	std::cout << "3" << std::endl;
		}
		else if(imu->calcAccel(imu->ay) < -0.01)
		{
			if(duty_cycle1 < MAXIMUM_DC)
				duty_cycle1 += DELTA;
			if(duty_cycle2 > MINUMUM_DC)
				duty_cycle2 -= DELTA;
			if(duty_cycle3 > MINUMUM_DC)
				duty_cycle3 -= DELTA;
			if(duty_cycle4 < MAXIMUM_DC)
				duty_cycle4 += DELTA;
//	    	std::cout << "4" << std::endl;
		}

		//Refresh every PWM
		pwm.setChlDuty(0,duty_cycle1);
		pwm.setChlDuty(1,duty_cycle2);
		pwm.setChlDuty(2,duty_cycle3);
		pwm.setChlDuty(3,duty_cycle4);

		//Just showing current DC
		std::cout << "Duty cycle: "	<< duty_cycle1 << "\t"
									<< duty_cycle2 << "\t"
									<< duty_cycle3 << "\t"
									<< duty_cycle4 << std::endl;

		//Repeat last steps every 5 ms
		usleep(5000);

	}


	return mraa::SUCCESS;

}












