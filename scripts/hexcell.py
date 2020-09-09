

class HexCell:
    def __init__(self, x=0, y=0, z=0):
        if x + y + z !=0:
            raise TypeError("Hex cell must meet x+y+z=0 requirement")
        self.cube = (int(x), int(y), int(z))

    def __hash__(self):
        return hash(self.cube)

    def __str__(self):
        return str(self.cube)

    def __repr__(self):
        return str(self.cube)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __abs__(self):
        return max(abs(self.x), abs(self.y), abs(self.z))  # cube distance

    def __add__(self, other):
        return HexCell(self.x+other.x, self.y+other.y, self.z+other.z)

    def __sub__(self, other):
        return HexCell(self.x-other.x, self.y-other.y, self.z-other.z)

    def __mul__(self, scalar):  # [i*scalar for i in self.cube]
        return HexCell(self.x*scalar, self.y*scalar, self.z*scalar)

    def __getitem__(self, idx):
        return self.cube[idx]
    
    @property
    def x(self):
        return self.cube[0]

    @property
    def y(self):
        return self.cube[1]

    @property
    def z(self):
        return self.cube[2]


'''
TODO: add more features in subclass
class ColorHexCell(HexCell):
    property:
    - self.status or self.color
        light green: visited
        dark green: visited twice or more
        light yellow: visitable (free space)
        light red: unvisitable/obstacle
        light grey: unknown
    - may use dictionary/hashing to improve indexing speed

    function:
    - self.boundary() returns boundary hex cube coordiantes
    - self.update_cells() updates cell status
'''
