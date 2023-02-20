import math

# ported from https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/src/quaternionFilters.cpp

PI = 3.14159265358979323846
GyroMeasError = PI * (40.0 / 180.0)
# gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
GyroMeasDrift = PI * (0.0  / 180.0)

beta = math.sqrt(3.0 / 4.0) * GyroMeasError
zeta = math.sqrt(3.0 / 4.0) * GyroMeasDrift

class MadgwickQuaternion:
    def __init__(self):
        self.__q = [ 1.0, 0.0, 0.0, 0.0 ]

    def get_q(self):
        return self.__q

    def update(self, aN, aE, aD, gN, gE, gD, mN, mE, mD, deltaT):
        global beta, zeta

        q = self.__q

        # norm
        # hx, hy, _2bx, _2bz
        # s1, s2, s3, s4
        # qDot1, qDot2, qDot3, qDot4

        #_2q1mN
        #_2q1mE
        #_2q1mD
        #_2q2mN
        #_4bx
        #_4bz
        _2q1 = 2.0 * q[0]
        _2q2 = 2.0 * q[1]
        _2q3 = 2.0 * q[2]
        _2q4 = 2.0 * q[3]
        _2q1q3 = 2.0 * q[0] * q[2]
        _2q3q4 = 2.0 * q[2] * q[3]

        q1q1 = q[0] * q[0]
        q1q2 = q[0] * q[1]
        q1q3 = q[0] * q[2]
        q1q4 = q[0] * q[3]
        q2q2 = q[1] * q[1]
        q2q3 = q[1] * q[2]
        q2q4 = q[1] * q[3]
        q3q3 = q[2] * q[2]
        q3q4 = q[2] * q[3]
        q4q4 = q[3] * q[3]

        # Normalize accelerometer measurement
        norm = math.sqrt(aN ** 2 + aE ** 2 + aD ** 2)
        if norm == 0.0:
            print('div 0 avoided 1')
            return  # Handle NaN
        norm = 1.0 / norm       # Use reciprocal for division
        aN *= norm
        aE *= norm
        aD *= norm

        # Normalize magnetometer measurement
        norm = math.sqrt(mN ** 2 + mE ** 2 + mD ** 2)
        if norm == 0.0:
            print('div 0 avoided 2')
            return  # Handle NaN
        norm = 1.0 / norm       # Use reciprocal for division
        mN *= norm
        mE *= norm
        mD *= norm

        # Reference direction of Earth's magnetic field
        _2q1mN = 2.0 * q[0] * mN
        _2q1mE = 2.0 * q[0] * mE
        _2q1mD = 2.0 * q[0] * mD
        _2q2mN = 2.0 * q[1] * mN
        hx = mN * q1q1 - _2q1mE * q[3] + _2q1mD * q[2] + mN * q2q2 + _2q2 * mE * q[2] + _2q2 * mD * q[3] - mN * q3q3 - mN * q4q4
        hy = _2q1mN * q[3] + mE * q1q1 - _2q1mD * q[1] + _2q2mN * q[2] - mE * q2q2 + mE * q3q3 + _2q3 * mD * q[3] - mE * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mN * q[2] + _2q1mE * q[1] + mD * q1q1 + _2q2mN * q[3] - mD * q2q2 + _2q3 * mE * q[3] - mD * q3q3 + mD * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - aN) + _2q2 * (2.0 * q1q2 + _2q3q4 - aE) - _2bz * q[2] * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + _2bx * q[2] * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - aN) + _2q1 * (2.0 * q1q2 + _2q3q4 - aE) - 4.0 * q[1] * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - aD) + _2bz * q[3] * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - aN) + _2q4 * (2.0 * q1q2 + _2q3q4 - aE) - 4.0 * q[2] * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - aD) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - aN) + _2q3 * (2.0 * q1q2 + _2q3q4 - aE) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + _2bx * q[1] * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)

        norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalize step magnitude
        if norm != 0.0:
            norm = 1.0/norm
            s1 *= norm
            s2 *= norm
            s3 *= norm
            s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q[1] * gN - q[2] * gE - q[3] * gD) - beta * s1
        qDot2 = 0.5 * (q[0] * gN + q[2] * gD - q[3] * gE) - beta * s2
        qDot3 = 0.5 * (q[0] * gE - q[1] * gD + q[3] * gN) - beta * s3
        qDot4 = 0.5 * (q[0] * gD + q[1] * gE - q[2] * gN) - beta * s4

        # Integrate to yield quaternion
        q[0] += qDot1 * deltaT
        q[1] += qDot2 * deltaT
        q[2] += qDot3 * deltaT
        q[3] += qDot4 * deltaT

        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])    # normalize quaternion
        norm = 1.0/norm
        q[0] = q[0] * norm
        q[1] = q[1] * norm
        q[2] = q[2] * norm
        q[3] = q[3] * norm
