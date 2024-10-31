import matplotlib.pyplot as plt
import time
import numpy as np
import yaml
from quad_copter import QuadCopter
from simulation import Simulation



def main():
    my_quad = QuadCopter()
    quad_list = [my_quad]
    sim = Simulation(quad_list)

    sim.run()


if __name__ == "__main__":
    main()