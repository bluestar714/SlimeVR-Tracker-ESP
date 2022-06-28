/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "globals.h"
//#include "helper_3dmath.h"

#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#else
#include "MPU6050_6Axis_MotionApps20.h"
#endif

#include "mpu6050sensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "calibration.h"
#include "GlobalVars.h"

//#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
//    #include "dmpmag.h"
//#endif
#include "mahony.h"
//#include "madgwick.h"

#define MAG_CORR_RATIO 0.02

#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s
#endif


void MPU6050Sensor::motionSetup()
{
    imu.initialize(addr);
    if (!imu.testConnection())
    {
        m_Logger.fatal("Can't connect to MPU6050 (0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        return;
    }

    m_Logger.info("Connected to MPU6050 (0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

#ifndef IMU_MPU6050_RUNTIME_CALIBRATION
    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::MPU6050:
            m_Calibration = sensorCalibration.data.mpu6050;
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }
#endif

    devStatus = imu.dmpInitialize();

    if (devStatus == 0)
    {
#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
        // We don't have to manually calibrate if we are using the dmp's automatic calibration
#else  // IMU_MPU6050_RUNTIME_CALIBRATION

        m_Logger.debug("Performing startup calibration of accel and gyro...");
        // Do a quick and dirty calibration. As the imu warms up the offsets will change a bit, but this will be good-enough
        delay(1000); // A small sleep to give the users a chance to stop it from moving

        imu.CalibrateGyro(6);
        imu.CalibrateAccel(6);
        imu.PrintActiveOffsets();
#endif // IMU_MPU6050_RUNTIME_CALIBRATION

        ledManager.pattern(50, 50, 5);

        // turn on the DMP, now that it's ready
        m_Logger.debug("Enabling DMP...");
        imu.setDMPEnabled(true);

        // TODO: Add interrupt support
        // mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        m_Logger.debug("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();

        working = true;
        configured = true;
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        m_Logger.error("DMP Initialization failed (code %d)", devStatus);
    }
}

void MPU6050Sensor::motionLoop()
{
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
#endif

#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    if (!dmpReady)
        return;

    if (imu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
        quaternion.set(-rawQuat.y, rawQuat.x, rawQuat.z, rawQuat.w);
        quaternion *= sensorOffset;   
    }

    Quat quat(-rawQuat.y,rawQuat.x,rawQuat.z,rawQuat.w);
    
    VectorFloat grav;
    imu.dmpGetGravity(&grav, &rawQuat);

    float Grav[] = {grav.x, grav.y, grav.z};
    
    if (correction.length_squared() == 0.0f) {
        //correction = getCorrection(Grav, Mxyz, quat);
        correction = getTmpCorrection(Grav, Mxyz, quat);
    } else {
        //Quat newCorr = getCorrection(Grav, Mxyz, quat);
        Quat newCorr = getTmpCorrection(Grav, Mxyz, quat);

        if(!__isnanf(newCorr.w)) {
            //correction = correction.slerp(newCorr, MAG_CORR_RATIO);
            correction = correction.slerp(newCorr, MAG_CORR_RATIO);
        }
    }

    quaternion = correction * quat;

#else //MAHONY or MADWICK
    unsigned long now = micros();
    unsigned long deltat = now - last; //seconds since last update
    last = now;
    getMPUScaled2();
    
    #if defined(_MAHONY_H_)
    //mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6);
    #elif defined(_MADGWICK_H_)
    //madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6);
    #endif
    quaternion.set(-q[2], q[1], q[3], q[0]);
#endif
    quaternion *= sensorOffset;

    m_Logger.info("q0: %f", q[0]);
    m_Logger.info("q1: %f", q[1]);
    m_Logger.info("q2: %f", q[2]);
    m_Logger.info("q3: %f", q[3]);
#if ENABLE_INSPECTION
        {
        Network::sendInspectionFusedIMUData(sensorId, quaternion);
        }
#endif

        if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
        {
            newData = true;
            lastQuatSent = quaternion;
        }
}


Quat MPU6050Sensor::getTmpQuatDCM(float* acc, float* mag){
    Vector3 Mv(mag[0], mag[1], mag[2]);
    Vector3 Dv(acc[0], acc[1], acc[2]);
    Dv.normalize();
    Vector3 Rv = Dv.cross(Mv);
    Rv.normalize();
    Vector3 Fv = Rv.cross(Dv);
    Fv.normalize();
    float q04 = 2*sqrt(1+Fv.x+Rv.y+Dv.z);
    m_Logger.info("getTmpresult: %f ", q04);
    return Quat(Rv.z-Dv.y,Dv.x-Fv.z,Fv.y-Rv.x,q04*q04/4).normalized();    
}

Quat MPU6050Sensor::getTmpCorrection(float* acc,float* mag,Quat quat)
{
    Quat magQ = getTmpQuatDCM(acc,mag);
    //dmp.w=DCM.z
    //dmp.x=DCM.y
    //dmp.y=-DCM.x
    //dmp.z=DCM.w
    Quat trans(magQ.x, magQ.y, magQ.w, magQ.z);
    Quat result = trans*quat.inverse();
    m_Logger.info("getTmpresult: %d ", result);
    return result;

}

void MPU6050Sensor::getMPUScaled2()
{
    float temp[3];
    int i;

    //m_Logger.info("I am loop man ");
#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
    int16_t ax, ay, az, gx, gy, gz;
    //imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //m_Logger.info("ax: (%f), ay: (%f), az: (%f), gx: (%f), gy: (%f), gz: (%f) ", ax, ay, az, gx, gy, gz);
    Gxyz[0] = ((float)gx - m_Calibration.G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - m_Calibration.G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - m_Calibration.G_off[2]) * gscale;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
        Axyz[0] = m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2];
        Axyz[1] = m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2];
        Axyz[2] = m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2];
    #else
        for (i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - m-Calibration.A_B[i]);
    #endif

#else
    //int16_t mx, my, mz;
    // with DMP, we just need mag data
    //imu.getMagnetometer(&mx, &my, &mz);
#endif
    Mxyz[0] = 0.0;
    Mxyz[1] = 0.0;
    Mxyz[2] = 0.0;
    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    //Mxyz[0] = 0.0;
    //Mxyz[1] = 0.0;
    //Mxyz[2] = -0.0;
    //apply offsets and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
        //temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
        //Mxyz[1] = m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2];
        //Mxyz[2] = m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2];
        //Mxyz[0] = m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2];
        Mxyz[0] = 0.0;
        Mxyz[1] = 0.0;
        Mxyz[2] = 0.0;
#else
        for (i = 0; i < 3; i++)
            Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
    #endif
}





void MPU6050Sensor::startCalibration(int calibrationType) {
    ledManager.on();

#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
    m_Logger.info("MPU is using automatic runtime calibration. Place down the device and it should automatically calibrate after a few seconds");

    // Lie to the server and say we've calibrated
    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_GYRO, 0);//was CALIBRATION_TYPE_INTERNAL_GYRO for some reason? there wasn't a point to this switch
        break;
    }
#else //!IMU_MPU6050_RUNTIME_CALIBRATION
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);

    imu.setDMPEnabled(false);
    imu.CalibrateGyro(6);
    imu.CalibrateAccel(6);
    imu.setDMPEnabled(true);

    m_Logger.debug("Gathered baseline gyro reading");
    m_Logger.debug("Starting offset finder");
    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        imu.CalibrateAccel(10);
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);//doesn't send calibration data anymore, has that been depricated in server?
        m_Calibration.A_B[0] = imu.getXAccelOffset();
        m_Calibration.A_B[1] = imu.getYAccelOffset();
        m_Calibration.A_B[2] = imu.getZAccelOffset();
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        imu.CalibrateGyro(10);
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_GYRO, 0);//doesn't send calibration data anymore
        m_Calibration.G_off[0] = imu.getXGyroOffset();
        m_Calibration.G_off[1] = imu.getYGyroOffset();
        m_Calibration.G_off[2] = imu.getZGyroOffset();
        break;
    }

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU6050;
    calibration.data.mpu6050 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    m_Logger.info("Calibration finished");
#endif // !IMU_MPU6050_RUNTIME_CALIBRATION

    ledManager.off();
}
