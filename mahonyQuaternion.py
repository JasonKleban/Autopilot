import math

# ported from https:#github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/src/quaternionFilters.cpp

# These are the free parameters in the Mahony filter and fusion scheme, Kp
# for proportional feedback, Ki for integral
Kp = 2.0 * 5.0
Ki = 0.0

class MahonyQuaternion:
    def __init__(self):
        self.__q = [ 1.0, 0.0, 0.0, 0.0 ]
        self.__eInt = [ 0.0, 0.0, 0.0 ]

    def get_q(self):
        return self.__q

    def update(self, aN, aE, aD, gN, gE, gD, mN, mE, mD, deltaT):
        global Kp, Ki

        q1, q2, q3, q4 = self.__q[0], self.__q[1], self.__q[2], self.__q[3]

        # Auxiliary variables to avoid repeated arithmetic
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
        norm = 1.0 / norm        # use reciprocal for division
        aN *= norm
        aE *= norm
        aD *= norm

        # Normalise magnetometer measurement
        norm = math.sqrt(mN * mN + mE * mE + mD * mD)
        if norm == 0.0: return # handle NaN
        norm = 1.0 / norm        # use reciprocal for division
        mN *= norm
        mE *= norm
        mD *= norm

        # Reference direction of Earth's magnetic field
        hx = 2.0 * mN * (0.5 - q3q3 - q4q4) + 2.0 * mE * (q2q3 - q1q4) + 2.0 * mD * (q2q4 + q1q3)
        hy = 2.0 * mN * (q2q3 + q1q4) + 2.0 * mE * (0.5 - q2q2 - q4q4) + 2.0 * mD * (q3q4 - q1q2)
        bx = math.sqrt((hx * hx) + (hy * hy))
        bz = 2.0 * mN * (q2q4 - q1q3) + 2.0 * mE * (q3q4 + q1q2) + 2.0 * mD * (0.5 - q2q2 - q3q3)

        # Estimated direction of gravity and magnetic field
        vx = 2.0 * (q2q4 - q1q3)
        vy = 2.0 * (q1q2 + q3q4)
        vz = q1q1 - q2q2 - q3q3 + q4q4
        wx = 2.0 * bx * (0.5 - q3q3 - q4q4) + 2.0 * bz * (q2q4 - q1q3)
        wy = 2.0 * bx * (q2q3 - q1q4) + 2.0 * bz * (q1q2 + q3q4)
        wz = 2.0 * bx * (q1q3 + q2q4) + 2.0 * bz * (0.5 - q2q2 - q3q3)

        # Error is cross product between estimated direction and measured direction of gravity
        ex = (aE * vz - aD * vy) + (mE * wz - mD * wy)
        ey = (aD * vx - aN * vz) + (mD * wx - mN * wz)
        ez = (aN * vy - aE * vx) + (mN * wy - mE * wx)
        if Ki > 0.0:
            self.__eInt[0] += ex      # accumulate integral error
            self.__eInt[1] += ey
            self.__eInt[2] += ez
        else:
            self.__eInt[0] = 0.0     # prevent integral wind up
            self.__eInt[1] = 0.0
            self.__eInt[2] = 0.0

        # Apply feedback terms
        gN = gN + Kp * ex + Ki * self.__eInt[0]
        gE = gE + Kp * ey + Ki * self.__eInt[1]
        gD = gD + Kp * ez + Ki * self.__eInt[2]

        # Integrate rate of change of quaternion
        pa = q2
        pb = q3
        pc = q4
        q1 = q1 + (-q2 * gN - q3 * gE - q4 * gD) * (0.5 * deltaT)
        q2 = pa + (q1 * gN + pb * gD - pc * gE) * (0.5 * deltaT)
        q3 = pb + (q1 * gE - pa * gD + pc * gN) * (0.5 * deltaT)
        q4 = pc + (q1 * gD + pa * gE - pb * gN) * (0.5 * deltaT)

        # Normalise quaternion
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        norm = 1.0 / norm
        self.__q[0] = q1 * norm
        self.__q[1] = q2 * norm
        self.__q[2] = q3 * norm
        self.__q[3] = q4 * norm
