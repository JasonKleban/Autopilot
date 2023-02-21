import math

# ported from https:#github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/src/quaternionFilters.cpp

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
        global beta

        q1, q2, q3, q4 = self.__q[0], self.__q[1], self.__q[2], self.__q[3]
        # norm
        # hx, hy, _2bx, _2bz
        # s1, s2, s3, s4
        # qDot1, qDot2, qDot3, qDot4

        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = math.sqrt(aN * aN + aE * aE + aD * aD)
        if norm == 0.0: return # handle NaN
        norm = 1.0/norm
        aN *= norm
        aE *= norm
        aD *= norm

        # Normalise magnetometer measurement
        norm = math.sqrt(mN * mN + mE * mE + mD * mD)
        if norm == 0.0: return # handle NaN
        norm = 1.0/norm
        mN *= norm
        mE *= norm
        mD *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2.0 * q1 * mN
        _2q1my = 2.0 * q1 * mE
        _2q1mz = 2.0 * q1 * mD
        _2q2mx = 2.0 * q2 * mN
        hx = mN * q1q1 - _2q1my * q4 + _2q1mz * q3 + mN * q2q2 + _2q2 * mE * q3 + _2q2 * mD * q4 - mN * q3q3 - mN * q4q4
        hy = _2q1mx * q4 + mE * q1q1 - _2q1mz * q2 + _2q2mx * q3 - mE * q2q2 + mE * q3q3 + _2q3 * mD * q4 - mE * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mD * q1q1 + _2q2mx * q4 - mD * q2q2 + _2q3 * mE * q4 - mD * q3q3 + mD * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - aN) + _2q2 * (2.0 * q1q2 + _2q3q4 - aE) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - aN) + _2q1 * (2.0 * q1q2 + _2q3q4 - aE) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - aD) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - aN) + _2q4 * (2.0 * q1q2 + _2q3q4 - aE) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - aD) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - aN) + _2q3 * (2.0 * q1q2 + _2q3q4 - aE) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mN) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mE) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mD)
        norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        norm = 1.0/norm
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gN - q3 * gE - q4 * gD) - beta * s1
        qDot2 = 0.5 * (q1 * gN + q3 * gD - q4 * gE) - beta * s2
        qDot3 = 0.5 * (q1 * gE - q2 * gD + q4 * gN) - beta * s3
        qDot4 = 0.5 * (q1 * gD + q2 * gE - q3 * gN) - beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * deltaT
        q2 += qDot2 * deltaT
        q3 += qDot3 * deltaT
        q4 += qDot4 * deltaT
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        norm = 1.0/norm
        self.__q[0] = q1 * norm
        self.__q[1] = q2 * norm
        self.__q[2] = q3 * norm
        self.__q[3] = q4 * norm
