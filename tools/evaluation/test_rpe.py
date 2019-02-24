import evaluation.evaluate_rpe as rpe
import numpy as np

theta = np.pi/6
l = [1000000000, 1, 2, 3, 0, 0, np.sin(theta/2), np.cos(theta/2)]
mat = rpe.transform44(l)
print(mat)