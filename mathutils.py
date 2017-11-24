import math


class Spline:

    def __init__(self, *points, tangents=None):
        assert tangents is None or len(points) == len(tangents)
        self.points = []
        for p, t in zip(points, tangents if tangents is not None else (0 for _ in points)):
            self.points.append((p, t))

    def __call__(self, item):
        assert isinstance(item, float) or isinstance(item, int)
        if item < 0:
            return self.points[0][0]
        i0 = int(math.floor(item))
        i1 = i0 + 1
        if i1 == len(self.points):
            return self.points[len(self.points) - 1][0]
        p0, m0 = self.points[i0]
        p1, m1 = self.points[i1]
        t = item - i0
        return h00(t) * p0 + h10(t) * m0 + h01(t) * p1 + h11(t) * m1


def h00(t): return 2 * t**3 - 3 * t**2 + 1


def h10(t): return t**3 - 2 * t**2 + t


def h01(t): return -2 * t**3 + 3 * t**2


def h11(t): return t**3 - t**2


def derivative(f, h=0.001):
    return lambda x: (f(x + h) - f(x - h)) / 2 / h

if __name__ == '__main__':
    s = Spline(0, 1, 5, 2, 1)
    ds = derivative(s)
    for i in range(0, 21):
        x = i / 5
        print(x, round(s(x), 2), round(ds(x), 2))
