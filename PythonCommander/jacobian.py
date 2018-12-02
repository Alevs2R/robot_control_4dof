from sympy import *

from elementary_transormations import *
q1, q2, q3, l1, l2, l3 = symbols('q1 q2 q3 l1 l2 l3')


# for artuculated RRR robot
def jacobian():
    # T0 = eye(4)
    # T0_1 = rz(q1)
    # T0_2 = T0_1 * tz(l1)
    # T0_3 = T0_2 * ry(q2)
    # T0_4 = T0_3 * tx(l2)
    # T0_5 = ry(-q3) * T0_4
    # T0_6 = T0_5 * tx(l3)
    #
    # x = simplify(T0_6[0, 3])
    # y = simplify(T0_6[1, 3])
    # z = simplify(T0_6[2, 3])

    x = l2 * cos(q2) * cos(q1) + \
        l3 * cos(q3) * cos(q1)

    y = l2 * cos(q2) * sin(q1) + \
        l3 * cos(q3) * sin(q1)

    z = l1 + l2 * sin(q2) - l3 * sin(q3)

    print(x)
    print(y)
    print(z)

    J = Matrix([
        [diff(x, q1), diff(x, q2), diff(x, q3)],
        [diff(y, q1), diff(y, q2), diff(y, q3)],
        [diff(z, q1), diff(z, q2), diff(z, q3)],
    ])

    print(simplify(J.inv()))
    return J

jacobian()