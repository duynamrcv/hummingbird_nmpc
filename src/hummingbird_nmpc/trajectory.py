import math
import numpy as np
import bisect
from mpl_toolkits.mplot3d import Axes3D 

class CubicSpline1D:
    """
    1D Cubic Spline class
    """

    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        """
        Calc `y` position for given `x`.

        if `x` is outside the data point's `x` range, return None.

        Returns
        -------
        y : float
            y position for given x.
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def calc_first_derivative(self, x):
        """
        Calc first derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        dy : float
            first derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, x):
        """
        Calc second derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        ddy : float
            second derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline3D:
    """
    Cubic CubicSpline3D class
    """

    def __init__(self, x, y, z):
        self.s = self.__calc_s(x, y, z)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)
        self.sz = CubicSpline1D(self.s, z)

    def __calc_s(self, x, y, z):
        dx = np.diff(x)
        dy = np.diff(y)
        dz = np.diff(z)
        self.ds = np.linalg.norm([dx, dy, dz], axis=0)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)
        z = self.sz.calc_position(s)

        return x, y, z

    def calc_yaw(self, s):
        """
        calc yaw

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw

# def main_2d():  # pragma: no cover
#     print("CubicSpline1D 2D test")
#     import matplotlib.pyplot as plt
#     x = [-2.5, 2.5, 7.5]
#     y = [0.7, 5, 0.0]
#     z = [5, 6, 4]
#     ds = 0.1  # [m] distance of each interpolated points

#     sp = CubicSpline3D(x, y, z)
#     s = np.arange(0, sp.s[-1], ds)

#     rx, ry, rz, ryaw = [], [], [], []
#     for i_s in s:
#         ix, iy, iz = sp.calc_position(i_s)
#         rx.append(ix)
#         ry.append(iy)
#         rz.append(iz)
#         ryaw.append(sp.calc_yaw(i_s))

#     plt.figure()
#     ax = plt.axes(projection='3d')
#     ax.plot(x, y, z, "xb", label="Data points")
#     ax.scatter(rx, ry, rz, "-r", label="Cubic spline path")
#     ax.grid(True)
#     # ax.axis('equal')
#     ax.set_xlabel("x[m]")
#     ax.set_ylabel("y[m]")
#     ax.set_zlabel("z[m]")
#     plt.legend()

#     plt.figure()
#     plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
#     plt.grid(True)
#     plt.legend()
#     plt.xlabel("line length[m]")
#     plt.ylabel("yaw angle[deg]")

#     plt.show()


# if __name__ == '__main__':
#     # main_1d()
#     main_2d()