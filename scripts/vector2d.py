from math import sqrt, atan2


class Vector2D:
    def __init__(self, x=0, y=0):
        self.point = [float(x), float(y)]

    def __hash__(self):
        return hash(tuple(self.point))

    def __str__(self):
        return str(self.point)

    def __repr__(self):
        return str(self.point)

    def __eq__(self, other):
        return self.point == other.point

    def __abs__(self):
        return sqrt( (self.x)**2 + (self.y)**2 )  # L-2 norm

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)

    def __getitem__(self, idx):
        return self.point[idx]

    def __setitem__(self, idx, value):
        self.point[idx] = value
    
    @property
    def x(self):
        return self.point[0]

    @property
    def y(self):
        return self.point[1]

    @property
    def angle(self):
        return atan2(self.y, self.x)

