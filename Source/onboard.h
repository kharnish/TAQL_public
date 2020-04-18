#pragma once
#ifndef onboard_h
#define onboard_h
#include "stateDerivative.h"
/*
#include "MPU9250.h" // IMU1 header
#include "LSM9DS1.h" // IMU2 header
#include "Util.h" // idk we just need it?

#include <string>
#include <memory>
*/
/*
#include "Navio/Navio2/PWM.h"
#include "Navio/Navio2/ADC_Navio2.h"
#include "Navio/Navio2/Led_Navio2.h"
#include "Navio/Navio2/LSM9DS1.h"
#include "Navio/Navio2/RCInput_Navio2.h"
#include "Navio/Navio2/RCOutput_Navio2.h"
#include "Navio/Navio2/RGBled.h"
#include "Navio/Common/ADC.h"
#include "Navio/Common/gpio.h"
#include "Navio/Common/I2Cdev.h"
#include "Navio/Common/InertialSensor.h"
#include "Navio/Common/Led.h"
#include "Navio/Common/MPU9250.h"
#include "Navio/Common/MS5611.h"
#include "Navio/Common/RCInput.h"
#include "Navio/Common/RCOutput.h"
#include "Navio/Common/SPIdev.h"
#include "Navio/Common/Ublox.h"
#include "Navio/Common/Util.h"
using namespace Navio;

//Auto Ptr copying from Dr.Johnson's code 

std::unique_ptr <InertialSensor> get_inertial_sensor() {

#ifdef IMU_MPU9250
	auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
	return ptr;
#endif
#ifdef IMU_LSM9DS1
	auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
	return ptr;
#endif
*/

void matMult(float a[12][12], float b[12], float mul[12]);
void matMult(float a[12][12], float b[12][12], float mul[12][12]);
void matMult(float a[12][12], float b[12][3], float mul[12][3]);
void matMult(float a[12][3], float b[3][12], float mul[12][12]);
void matMult(float a[12][3], float b[3][3], float mul[12][3]);

void matScale(float a[12][12], float con, float mul[12][12]);
void matScale(float a[12], float con, float mul[12]);

void matAdd(float a[12][12], float b[12][12], float mul[12][12]);
void matAdd(float a[3][3], float b[3][3], float mul[3][3]);
void matAdd(float a[12], float b[12], float mul[12]);

void matSub(float a[12], float b[12], float mul[12]);
void matInv(float a[3][3], float mul[3][3]);


void vehicleController(float* stick, float* measData, float* powerCmd) {
	float Ks[3] = { 3.14, 3.14, 3.14 }; // Ks is a feedforward gain
	float Kp[3] = { .1, .1, .1 }; // Kp is a proportional feedback gain

	float motorCmd[4] = {};

	float pqrFiltered[3] = {};
	for (int i = 0; i < 3; i++) {
		pqrFiltered[i] = (measData[P1 + i] + measData[P2 + i]) / float(2); // simply avg the two different IMUs together. DOES NOT account for biases
	}

	for (int i = 0; i < 3; i ++) {
		motorCmd[i] = Kp[i]*(Ks[i]*stick[i] - pqrFiltered[i]);
	}
	motorCmd[3] = stick[THRUST];

	powerCmd[0] = float(0.125)*(float(4) - motorCmd[0] + motorCmd[1] + motorCmd[2] + motorCmd[3]);
	powerCmd[1] = float(0.125)*(float(4) + motorCmd[0] - motorCmd[1] + motorCmd[2] + motorCmd[3]);
	powerCmd[2] = float(0.125)*(float(4) + motorCmd[0] + motorCmd[1] - motorCmd[2] + motorCmd[3]);
	powerCmd[3] = float(0.125)*(float(4) - motorCmd[0] - motorCmd[1] - motorCmd[2] + motorCmd[3]);
}


void navigation(float* measData, float* powerCmd, float* stateEst) {
	// recieves sensor (measured) data and outputs estimated states from Kalman Filter
	// helpful reference: http://robotsforroboticists.com/kalman-filtering/


	// measData: // {x, y, z1, z2, p1, q1, r1, ax1, ay1, az1, p2, q2, r2, ax2, ay2, az2}
	// cam: x, y, z1
	// sonar: z2
	// imu1: p1, q1, r1, ax1, ay1, az1
	// imu2: p2, q2, r2, ax2, ay2. az2


	bool takeMeasure = true;
	float dt = (float)0.05;

	float accel[3] = {}; // this is ax, ay, az which we measure

	// TODO: we need to figure out this matrix so that stateNew = A*statePast, to give x = A*statePast + B*action
	float A[12][12] = {};
	A[0][6] = cos(stateEst[PSI]);
	A[0][7] = -sin(stateEst[PSI]);
	A[1][6] = sin(stateEst[PSI]);
	A[1][7] = cos(stateEst[PSI]);
	A[2][8] = 1;
	A[9][9] = 1;
	A[4][10] = 1;
	A[5][11] = 1;
	A[6][4] = -g;
	A[7][3] = g;
	A[8][3] = -g;
	A[8][4] = -g;

	// float B[12][4] = {};

	float ATransp[12][12] = {};
	for (int i = 0; i < 12; ++i)
	{
		for (int j = 0; j < 12; ++j)
		{
			ATransp[j][i] = A[i][j];
		}
	}

	// C isn't just an identity matrix, because we're not taking a measurement of every state, only count X, Y, and Z
	float C[3][12] = {};
	C[0][X] = 1;
	C[1][Y] = 1;
	C[2][Z] = 1;
	float CTransp[12][3] = {};
	CTransp[X][0] = 1;
	CTransp[Y][1] = 1;
	CTransp[Z][2] = 1;

	float pMat[12][12] = {};
	float Qc[12][12] = {};	// expected values of disturbance (w^2)
	float Rc[3][3] = {};	// expected values of noise variance (v^2)
	float Kc[12][3] = {};	// how much we want to weight the error
	float I[12][12] = {};	// identity matrix

	float totalXEstDot[12] = {};
	float xEstDot1 = 0, xEstDot2 = 0, xEstDot3 = 0, xEstDot4 = 0;
	float xEst[12] = {};
	float xEstNew[12] = {};
	float stateMeas[12] = { };
	float stateEstPast[12] = {};

	for (int i = 0; i < 12; i++)
	{
		pMat[i][i] = 1;
		Qc[i][i] = 1;
		I[i][i] = 1;
		stateEstPast[i] = stateEst[i];
		xEst[i] = stateEst[i];
	}
	for (int i = 0; i < 3; i++) {
		Rc[i][i] = 1;
		stateMeas[i] = measData[i]; // only include the X, Y, Z data
	}

	// Start the actual agorithm
	for (int i = 0; i < 12; i++)
	{
		// RK4 algorithm
		float k[4][12] = {}; // here, k is xEstDot

		stateDerivative(stateEstPast, k[0], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			stateEst[i] = stateEstPast[i] + k[0][i] * dt / float(2);
		stateDerivative(stateEstPast, k[1], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			stateEst[i] = stateEstPast[i] + k[1][i] * dt / float(2);
		stateDerivative(stateEstPast, k[2], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			stateEst[i] = stateEstPast[i] + k[2][i] * dt;
		stateDerivative(stateEstPast, k[3], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			totalXEstDot[i] = (k[0][i] + float(2) * k[1][i] + float(2) * k[2][i] + k[3][i]) / float(6);
		// but don't integrate yet until we add the measurements
	}

	// predictions
	float temp1[12][12] = {};
	float temp2[12][12] = {};
	float temp3[12][12] = {};

	float pMatTemp[12][12] = {};
	float pDot[12][12] = {};
	float pDot1[12][12] = {};
	float pDot2[12][12] = {};
	float pDot3[12][12] = {};
	float pDot4[12][12] = {};

	matMult(A, pMat, temp1);
	matMult(temp1, ATransp, temp2);
	matAdd(temp2, Qc, pDot1); 	// pDot1 = A*P*A' + Q

	matScale(pDot1, dt / float(2), temp1);
	matAdd(pMat, temp1, temp2);
	matMult(A, temp2, temp1);
	matMult(temp1, ATransp, temp2);
	matAdd(temp2, Qc, pDot2); // pDot2 = A*(P + pDot1*dt/2)*A' + Q

	matScale(pDot2, dt / float(2), temp1);
	matAdd(pMat, temp1, temp2);
	matMult(A, temp2, temp1);
	matMult(temp1, ATransp, temp2);
	matAdd(temp2, Qc, pDot3); // pDot3 = A*(P + pDot2*dt/2)*A' + Q

	matScale(pDot3, dt, temp1);
	matAdd(pMat, temp1, temp2);
	matMult(A, temp2, temp1);
	matMult(temp1, ATransp, temp2);
	matAdd(temp2, Qc, pDot4); // pDot4 = A*(P + pDot3*dt)*A' + Q

	matScale(pDot2, 2, temp2);			// t2 = 2*pDot2
	matAdd(pDot1, temp2, temp1);		// t1 = pDot1 + 2*pDot2
	matScale(pDot3, 2, temp2);			// t2 = 2*pDot3 
	matAdd(temp1, temp2, temp3);		// t3 = pDot1 + 2*pDot2 + 2*pDot3
	matAdd(temp3, pDot4, temp1);		// t1 = pDot1 + 2*pDot2 + 2*pDot3 + pDot4
	matScale(temp1, float(1 / 6), pDot);// pDot = (1/6) * (pDot1 + 2*pDot2 + 2*pDot3 + pDot4)

	matScale(pDot, dt, temp1);
	matAdd(pMat, temp1, pMatTemp); // pMatTemp = pMat + pDot*dt. This is the first pMat before the measurements


	if (takeMeasure = true)
	{
		// TODO: ask Johnson about how we take in the acceleration measurements if they're not an actual state
		// treat IMU data as part of the process model, the dynamics as opposed to the updates. We can just pretend it's the actual process. Have the processs driven by the inertial data. But there's also process noise which is now measurement noise

		// correct the Kalman Filter, would be running onboard
		// make estimate of what the measures would be from xest = C*transp(Y)	
		float yEst[3] = {};
		matMult(C, xEst, yEst);
		float stateMeasEst[12] = { yEst[0], yEst[1], yEst[2], 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		// calculate Kc - note: Vitor calls 'C' as 'H' in his example
		float temp6a[12][3] = {};
		float temp6b[3][12] = {};
		float temp6c[3][3] = {};
		float temp6d[3][3] = {};
		matMult(pMatTemp, CTransp, temp6a);	// ta = P * C'
		matMult(C, pMatTemp, temp6b);		// tb = C * P
		matMult(temp6b, CTransp, temp6c);	// tc = C * P * C'
		matAdd(temp6c, Rc, temp6d);			// td = C * P * C' + Rc
		matInv(temp6d, temp6c);				// tc = inv(C * pDot * C' + Rc)
		matMult(temp6a, temp6c, Kc);		// Kc = P * C' * inv(C * P * C' + Rc)

		// Apply the measurement's contribution to xEstDot
		// this is the same as xhat = xhat + K*residual, where residual = C*(state - lastState)
		float temp12a[12] = {};
		float temp12b[12] = {};
		matMult(Kc, C, temp1);						// t1 = Kc * C
		matSub(stateMeas, stateMeasEst, temp12a);	// ta = stateMeas - stateMeasEst
		matMult(temp1, temp12a, temp12b);			// Kc * C * (stateMeas - stateMeasEst) [12x12 * 12x0]
		matAdd(totalXEstDot, temp12b, temp12a);		// newTotalXEstDot = totalXEstDot + Kc * C * (stateMeas - stateMeasEst)

		// correct P estimate
		matMult(Kc, C, temp1);			// t1 = Kc * C
		matScale(temp1, -1, temp2);		// t2 = -Kc*C
		matAdd(I, temp2, temp1);		// t1 = I + (-Kc*C)
		matMult(temp2, pMatTemp, pMat); // pMatNew = (I - Kc * C) * pMat
	}

	// actually finish integrating
	float temp12[12] = {};
	matScale(totalXEstDot, dt, temp12); // t1 = totalXEstDot * dt
	matAdd(xEst, temp12, xEstNew);		// xEstNew = xEstOld + totalXEstDot * dt
	for (int i = 0; i < 12; i++) {
		// stateEst[i] = xEstNew[i] // this is just commented out so it doesn't break anything
	}
}

void guidance(float* stateEst, float* stateDes, float* xyzpsiErrPast, float* xyzpsiErrSum, float* stick) {
	float Ks = float(3.14) / float(6.0) / float(100.0);
	float Kp = float(1);
	float Ki = float(0.001);
	float Kd = float(-0.1);

	stateDes[PSI] = (stateDes[Y] - stateEst[Y]) / (stateDes[X] - stateEst[X]);

	float xyzpsiErr[4] = {};
	xyzpsiErr[0] = stateDes[X] - stateEst[X];
	xyzpsiErr[1] = stateDes[Y] - stateEst[Y];
	xyzpsiErr[2] = stateDes[Z] - stateEst[Z];
	xyzpsiErr[3] = stateDes[PSI] - stateEst[PSI];

	for (int i = 0; i < 4; i++) {
		xyzpsiErrSum[i] += xyzpsiErr[i];
	}

	stateDes[P] =   Kp*xyzpsiErr[1] + Ki*xyzpsiErrSum[1] + Kd*(xyzpsiErr[1] - xyzpsiErrPast[1]);
	stateDes[Q] = -(Kp*xyzpsiErr[0] + Ki*xyzpsiErrSum[0] + Kd*(xyzpsiErr[0] - xyzpsiErrPast[0]));
	stateDes[R] =   Kp*xyzpsiErr[3] + Ki*xyzpsiErrSum[3] + Kd*(xyzpsiErr[3] - xyzpsiErrPast[3]);
	stateDes[W] =   Kp*xyzpsiErr[2] + Ki*xyzpsiErrSum[2] + Kd*(xyzpsiErr[2] - xyzpsiErrPast[2]);

	stateDes[P] = 0; // temp
	stateDes[Q] = 0; // temp
	stateDes[R] = 0; // temp

	float pqrwErr[4] = {};
	pqrwErr[0] = stateDes[P] - stateEst[P];
	pqrwErr[1] = stateDes[Q] - stateEst[Q];
	pqrwErr[2] = stateDes[R] - stateEst[R];
	pqrwErr[3] = stateDes[W] - stateEst[W];

	for (int i = 0; i < 4; i++) {
		stick[i] = Ks * pqrwErr[i];
	}

	for (int i = 0; i < 4; i++) {
		xyzpsiErrPast[i] = xyzpsiErr[i];
	}
}

void readIMU(float* imuData) {
	/*
	//imu initializer
	printf( "Init IMU and Mag\n" );
	auto imu = get_inertial_sensor();    
	if( !imu ) {
		printf( "IMU not found\n" );
        return EXIT_FAILURE;
    }
	if( !imu->probe() ) {
        printf( "IMU not enabled\n" );
        return EXIT_FAILURE;
    }
    imu->initialize();	

	//imu upadte
	imu->update();
	imu->read_gyroscope( &wx, &wy, &wz );
	wxTotal += wx;
	wyTotal += wy;
	wzTotal += wz;
	wxTotalAuto += wx;
	wyTotalAuto += wy;
	wzTotalAuto += wz;
	imu->read_accelerometer( &ax, &ay, &az );
	axTotalAuto += ax;
	ayTotalAuto += ay;
	azTotalAuto += az;
	IMUsamples++;
	IMUsamplesAuto++;

	// imu 1 MPU9250
	imuData[P1] = ax;
	imuData[Q1] = ay;
	imuData[R1] = az;
	imuData[AX1] = wx;
	imuData[AY1] = wy;
	imuData[AZ1] = wz;

	//imu 2 LSM9DS1
	imuData[P2] = 0;
	imuData[Q2] = 0;
	imuData[R2] = 0;
	imuData[AX2] = 0;
	imuData[AY2] = 0;
	imuData[AZ2] = 0;
	*/
}

void readSonar(float* stateMeas) {

}

void readCam(float* stateMeas) {

}

void readRC(float* stick) {
	stick[ROLL] = 0;   // -1 to 1
	stick[PITCH] = 0;  // -1 to 1
	stick[YAW] = 0;    // -1 to 1
	stick[THRUST] = 0; //  0 to 1
}

void matMult(float a[12][12], float b[12], float mul[12]) {
	// Matrix multiplication of a 12x12 and 12x0 matrix

	int r = 12, c = 12;

	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			for (int k = 0; k < c; k++)
			{
				mul[i] += a[i][k] * b[k];
			}
		}
	}
}
void matMult(float a[12][12], float b[12][12], float mul[12][12]) {
	// Matrix multiplication of two 12x12 matrices

	int r = 12, c = 12;

	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			for (int k = 0; k < c; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult(float a[12][12], float b[12][3], float mul[12][3]) {
	// Matrix multiplication of a 12x12 and 12x3 matrix = 12x3 matrix
	int r = 12, c = 3;

	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			for (int k = 0; k < c; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult(float a[12][3], float b[3][12], float mul[12][12]) {
	// Matrix multiplication of a 12x3 and 3x12 matrix

	int r = 12, c = 12;

	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			for (int k = 0; k < c; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult(float a[12][3], float b[3][3], float mul[12][3]) {
	// Matrix multiplication of a 12x3 and 3x3 matrix = 12x3 matrix
	int r = 12, c = 3;

	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			for (int k = 0; k < c; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matScale(float a[12][12], float con, float mul[12][12]) {
	// Scale a 12x12 matrix by a constant
	int r = 12, c = 12;
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			mul[i][j] = a[i][j] * con;
		}
	}
}
void matScale(float a[12], float con, float mul[12]) {
	// Scale a 12x12 matrix by a constant
	int r = 12;
	for (int i = 0; i < r; i++)
	{
		mul[i] = a[i] * con;
	}
}
void matAdd(float a[12][12], float b[12][12], float mul[12][12]) {
	// Matrix addition of two 12x12 matrices
	int r = 12, c = 12;
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			mul[i][j] = a[i][j] + b[i][j];
		}
	}
}
void matAdd(float a[3][3], float b[3][3], float mul[3][3]) {
	// Matrix addition of two 6x6 matrices
	int r = 6, c = 6;
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			mul[i][j] = a[i][j] + b[i][j];
		}
	}
}
void matAdd(float a[12], float b[12], float mul[12]) {
	// Matrix addition of two 12x12 matrices
	int r = 12, c = 1;
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			mul[i] = a[i] + b[i];
		}
	}
}
void matSub(float a[12], float b[12], float mul[12]) {
	// Matrix subtraction of two 12x12 matrices
	int r = 12, c = 1;
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			mul[i] = a[i] - b[i];
		}
	}
}
void matInv(float a[3][3], float mul[3][3]) {
	// Matrix invserion of a 3x3 matrix
	int r = 3, c = 3;
	float det = 0;
	float aT[3][3] = { };

	for (int i = 0; i < 3; i++)
	{
		det = det + (a[0][i] * (a[1][(i + 1) % 3] * a[2][(i + 2) % 3] - a[1][(i + 2) % 3] * a[2][(i + 1) % 3]));
	}

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			aT[j][i] = a[i][j];
		}
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			mul[i][j] = ((aT[(i + 1) % 3][(j + 1) % 3] * aT[(i + 2) % 3][(j + 2) % 3]) -
				(aT[(i + 1) % 3][(j + 2) % 3] * aT[(i + 2) % 3][(j + 1) % 3])) / det;
		}
	}

}
#endif // !"onboard.h"