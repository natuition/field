from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import pandas as pd
import pickle

dataset_train = pd.read_csv('dataset_normalized.csv')
X_train = dataset_train.iloc[:, 0:3].values #prend les 3 premières colonnes, tout px
Y_train = dataset_train.iloc[:, 3:6].values #les 3 dernières, tout mm
degree = 3
poly_features = PolynomialFeatures(degree)
X_train_poly = poly_features.fit_transform(X_train) #charge nos données pixels dans la matrice
poly_model = LinearRegression() #modèle de régression linéaire
poly_model.fit(X_train_poly, Y_train) #on charge nos données, x training data, y target values
filename = 'zone_model_'+str(degree)+'.sav'
pickle.dump(poly_model, open(filename, 'wb')) #sérialise l'objet dans le fichier
print('Done')