import numpy as np
a = np.random.choice([0, 1], size=(10,5), p=[1./3, 2./3])
print(a)
print(a.shape)

db = np.sum(a, axis=0)
print(db)
print(db.shape)