import math

# ported from https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/src/quaternionFilters.cpp

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

        q = self.__q
        eInt = self.__eInt

        # norm
        # hx, hy, bx, bz
        # vx, vy, vz, wx, wy, wz
        # ex, ey, ez
        # pa, pb, pc

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
        if norm == 0.0: return  # Handle NaN
        norm = 1.0 / norm       # Use reciprocal for division
        aN *= norm
        aE *= norm
        aD *= norm

        # Normalize magnetometer measurement
        norm = math.sqrt(mN ** 2 + mE ** 2 + mD ** 2)
        if norm == 0.0: return  # Handle NaN
        norm = 1.0 / norm       # Use reciprocal for division
        mN *= norm
        mE *= norm
        mD *= norm

        # Reference direction of Earth's magnetic field
        hx = 2.0 * mN * (0.5 - q3q3 - q4q4) + 2.0 * mE * (q2q3 - q1q4) + 2.0 * mD * (q2q4 + q1q3)
        hy = 2.0 * mN * (q2q3 + q1q4) + 2.0 * mE * (0.5 - q2q2 - q4q4) + 2.0 * mD * (q3q4 - q1q2)
        bx = math.sqrt((hx ** 2) + (hy ** 2))
        bz = 2.0 * mN * (q2q4 - q1q3) + 2.0 * mE * (q3q4 + q1q2) + 2.0 * mD * (0.5 - q2q2 - q3q3)

        # Estimated direction of gravity and magnetic field
        vx = 2.0 * (q2q4 - q1q3)
        vy = 2.0 * (q1q2 + q3q4)
        vz = q1q1 - q2q2 - q3q3 + q4q4
        wx = 2.0 * bx * (0.5 - q3q3 - q4q4) + 2.0 * bz * (q2q4 - q1q3)
        wy = 2.0 * bx * (q2q3 - q1q4) + 2.0 * bz * (q1q2 + q3q4)
        wz = 2.0 * bx * (q1q3 + q2q4) + 2.0 * bz * (0.5 - q2q2 - q3q3)

        #print(f'intermediate: {vx:+4.2f} {vy:+4.2f} {vz:+4.2f} {wx:+4.2f} {wy:+4.2f} {wz:+4.2f}')

        # Error is cross product between estimated direction and measured direction of gravity
        ex = (aE * vz - aD * vy) + (mE * wz - mD * wy)
        ey = (aD * vx - aN * vz) + (mD * wx - mN * wz)
        ez = (aN * vy - aE * vx) + (mN * wy - mE * wx)
        if Ki > 0.0:
            eInt[0] += ex      # accumulate integral error
            eInt[1] += ey
            eInt[2] += ez
        else:
            eInt[0] = 0.0      # prevent integral wind up
            eInt[1] = 0.0
            eInt[2] = 0.0

        # Apply feedback terms
        gN = gN + Kp * ex + Ki * eInt[0]
        gE = gE + Kp * ey + Ki * eInt[1]
        gD = gD + Kp * ez + Ki * eInt[2]

        # Integrate rate of change of quaternion
        pa = q[1]
        pb = q[2]
        pc = q[3]
        q[0] = q[0] + (-q[1] * gN - q[2] * gE - q[3] * gD) * (0.5 * deltaT)
        q[1] = pa + (q[0] * gN + pb * gD - pc * gE) * (0.5 * deltaT)
        q[2] = pb + (q[0] * gE - pa * gD + pc * gN) * (0.5 * deltaT)
        q[3] = pc + (q[0] * gD + pa * gE - pb * gN) * (0.5 * deltaT)

        # Normalize quaternion
        norm = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
        norm = 1.0 / norm
        q[0] = q[0] * norm
        q[1] = q[1] * norm
        q[2] = q[2] * norm
        q[3] = q[3] * norm
