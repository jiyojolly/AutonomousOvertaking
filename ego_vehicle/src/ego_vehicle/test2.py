from math import sqrt
from joblib import Parallel, delayed
ans = Parallel(n_jobs=2)(delayed(sqrt)(i ** 2) for i in range(10))
ans2 = [sqrt(i ** 2) for i in range(10)]
print(ans)
print(ans2)