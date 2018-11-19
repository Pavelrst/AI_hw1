import numpy as np
from matplotlib import pyplot as plt


def calc_denominator(alpha, t) -> float:
    assert alpha > 0 and t > 0
    ret = 0.0
    for x_h in X:
        ret += (x_h/alpha)**(-1/t)
    return ret


X = np.array([400, 450, 900, 390, 550])

# Our code below
alpha = min(X)
T = np.linspace(0.01, 5, 100)
P = np.zeros((len(T), len(X)))

# calculate all probabilities
for i, t in enumerate(T):
    for j, x in enumerate(X):
        P[i, j] = ((x/alpha) ** (-1 / t)) / calc_denominator(alpha, t)

# our code above
print(P)

for j in range(len(X)):
    plt.plot(T, P[:, j], label=str(X[j]))

plt.xlabel("T")
plt.ylabel("P")
plt.title("Probability as a function of the temperature")
plt.legend()
plt.grid()
plt.show()
exit()
