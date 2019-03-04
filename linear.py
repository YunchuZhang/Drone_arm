import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
data = np.array([[1.185,0.2],[1.185,0.15],[1.185,0.1],[1.185,0.15],[1.185,0.2],
	[1.22,0.2],[1.22,0.3],[1.22,0.2],[1.22,0.25],[1.22,0.2],
	[1.255,0.3],[1.255,0.35],[1.255,0.35],[1.255,0.35],[1.255,0.3],
	[1.305,0.5],[1.305,0.5],[1.305,0.5],[1.305,0.4],[1.305,0.47],
	[1.33,0.65],[1.33,0.65],[1.33,0.6],[1.33,0.575],
	[1.366,0.9],[1.366,0.8],[1.366,0.78],
	[1.4,1.15],[1.4,0.9],[1.4,0.88]
	])
print(data)
X = (data[:,0]).reshape(-1, 1)
y = (data[:,1]).reshape(-1, 1)

model = LinearRegression()
model.fit(X, y)
print(model.coef_)
y_pre = model.predict(X)

print(model.score(X,y))

# Plot outputs
plt.scatter(X, y,  color='black')
plt.plot(X, y_pre, color='blue', linewidth=3)

plt.xticks(())
plt.yticks(())

plt.show()