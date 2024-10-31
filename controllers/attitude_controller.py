import numpy as np

class AttittudeController:
    def __init__(self) -> None:
        self.u1 = 0
        self.u2 = 0
        self.u3 = 0
        self.u4 = 0

    def output(self):
        return np.array([self.u1, self.u2, self.u3, self.u4]).reshape(4, 1)

    