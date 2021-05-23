import math
import pybullet as p
from pyquaternion.quaternion import Quaternion


def change_of_basis(orientation: Quaternion, conventions: Quaternion) -> Quaternion:
    return conventions.__mul__(orientation).__mul__(conventions.inverse)


def convert_quaternion_to_bullet(q: Quaternion):
    return [q.x, q.y, q.z, q.w]


def euler_to_quaternion(euler) -> Quaternion:
    q = p.getQuaternionFromEuler(euler)
    return Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])


def convert(xyz, conventions):
    return p.getEulerFromQuaternion(convert_quaternion_to_bullet(change_of_basis(euler_to_quaternion(xyz), euler_to_quaternion(conventions))))


q0 = (0, 0, math.pi / 2)
q1 = (0, math.pi / 2, math.pi / 2)
q2 = (math.pi / 2, 0, math.pi / 2)
q3 = (math.pi / 2, 0, 0)
q4 = (math.pi / 2, math.pi / 2, 0)

q5 = (0, 0, math.pi)
q6 = (0, math.pi, math.pi)
q7 = (math.pi, 0, math.pi)
q8 = (math.pi, 0, 0)
q9 = (math.pi, math.pi, 0)

q10 = (0, 0, -math.pi / 2)
q11 = (0, -math.pi / 2, -math.pi / 2)
q12 = (-math.pi / 2, 0, -math.pi / 2)
q13 = (-math.pi / 2, 0, 0)
q14 = (-math.pi / 2, -math.pi / 2, 0)

q15 = (0, -math.pi / 2, math.pi / 2)
q16 = (-math.pi / 2, 0, math.pi / 2)
q17 = (-math.pi / 2, 0, 0)
q18 = (-math.pi / 2, math.pi / 2, 0)

q19 = (0, math.pi / 2, -math.pi / 2)
q20 = (math.pi / 2, 0, -math.pi / 2)
q21 = (math.pi / 2, 0, 0)
q22 = (math.pi / 2, -math.pi / 2, 0)

q23 = (0, -math.pi / 2, -math.pi)
q24 = (-math.pi / 2, 0, -math.pi)
q25 = (-math.pi / 2, 0, 0)
q26 = (-math.pi / 2, -math.pi, 0)

q27 = (0, math.pi, math.pi / 2)
q28 = (math.pi, 0, math.pi / 2)
q29 = (math.pi, 0, 0)
q30 = (math.pi, math.pi / 2, 0)

# x -90 z 180
q31 = (-math.pi/2, 0, math.pi)

input0 = (0, 0, -math.pi / 2)
output0 = (0, -math.pi / 2, 0)
input1 = (-math.pi / 2, 0, math.pi / 2)
output1 = (-math.pi / 2, math.pi / 2, 0)
input2 = (0, -math.pi / 2, 0)
output2 = (0, 0, math.pi / 2)

print("output: ", output0, convert(input0, q16), output0 == convert(input0, q16))
print("output: ", output1, convert(input1, q16), output1 == convert(input1, q16))
print("output: ", output2, convert(input2, q16), output2 == convert(input2, q16))

print("output: ", output0, convert(input0, q31), output0 == convert(input0, q31))
print("output: ", output1, convert(input1, q31), output1 == convert(input1, q31))
print("output: ", output2, convert(input2, q31), output2 == convert(input2, q31))

# print("output: ", output0, convert(input0, q0), output0 == convert(input0, q0))
# print("output: ", output0, convert(input0, q1), output0 == convert(input0, q1))
# print("output: ", output0, convert(input0, q2), output0 == convert(input0, q2))

print("output: ", output1, convert(input1, q4), output1 == convert(input1, q4))
print("output: ", output2, convert(input2, q5), output2 == convert(input2, q5))
print("output: ", output0, convert(input0, q6), output0 == convert(input0, q6))
print("output: ", output0, convert(input0, q7), output0 == convert(input0, q7))
print("output: ", output0, convert(input0, q8), output0 == convert(input0, q8))
print("output: ", output0, convert(input0, q9), output0 == convert(input0, q9))
print("output: ", output1, convert(input1, q10), output1 == convert(input1, q10))
print("output: ", output2, convert(input2, q11), output2 == convert(input2, q11))
print("output: ", output0, convert(input0, q12), output0 == convert(input0, q12))
print("output: ", output0, convert(input0, q13), output0 == convert(input0, q13))
print("output: ", output1, convert(input1, q14), output1 == convert(input1, q14))
print("output: ", output2, convert(input2, q15), output2 == convert(input2, q15))

print("output: ", output0, convert(input0, q17), output0 == convert(input0, q17))
print("output: ", output0, convert(input0, q18), output0 == convert(input0, q18))
print("output: ", output0, convert(input0, q19), output0 == convert(input0, q19))
print("output: ", output1, convert(input1, q20), output1 == convert(input1, q20))

print("output: ", output2, convert(input2, q21), output2 == convert(input2, q21))
print("output: ", output0, convert(input0, q22), output0 == convert(input0, q22))
print("output: ", output0, convert(input0, q23), output0 == convert(input0, q23))
print("output: ", output1, convert(input1, q24), output1 == convert(input1, q24))
print("output: ", output2, convert(input2, q25), output2 == convert(input2, q25))
print("output: ", output0, convert(input0, q26), output0 == convert(input0, q26))
print("output: ", output0, convert(input0, q27), output0 == convert(input0, q27))
print("output: ", output0, convert(input0, q28), output0 == convert(input0, q28))
print("output: ", output0, convert(input0, q29), output0 == convert(input0, q29))
print("output: ", output1, convert(input1, q30), output1 == convert(input1, q30))

print("output: ", output0, convert(input0, q3), output0 == convert(input0, q3))
print("output: ", output1, convert(input1, q3), output1 == convert(input1, q3))
print("output: ", output2, convert(input2, q3), output2 == convert(input2, q3))