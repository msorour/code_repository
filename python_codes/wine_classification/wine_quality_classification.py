import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from keras.models import Sequential
from keras.layers import Dense
from sklearn.metrics import confusion_matrix, precision_score, recall_score, f1_score, cohen_kappa_score
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import r2_score
from keras.optimizers import RMSprop, SGD

# Read in white wine data 
white = pd.read_csv("winequality-white.csv", sep=';')
# Read in red wine data 
red = pd.read_csv("winequality-red.csv", sep=';')


#######################
# Add `type` column to `red` with value 1
red['type'] = 1
# Add `type` column to `white` with value 0
white['type'] = 0
# Append `white` to `red`
wines = red.append(white, ignore_index=True)

####################
# Isolate target labels
Y = wines.quality

# Isolate data
X = wines.drop('quality', axis=1)

#############################
# Scale the data with `StandardScaler`
X = StandardScaler().fit_transform(X)
#####################
# Initialize the constructor
model = Sequential()

# Add input layer 
model.add(Dense(128, input_dim=12, activation='relu'))
#model.add(Dense(64, activation='relu'))
    
# Add output layer 
model.add(Dense(1))

######################
seed = 7
np.random.seed(seed)
kfold = StratifiedKFold(n_splits=5, shuffle=True, random_state=seed)
rmsprop = RMSprop(lr=0.002)
sgd=SGD(lr=0.1)
for train, test in kfold.split(X, Y):
    model = Sequential()
    model.add(Dense(64, input_dim=12, activation='relu'))
    model.add(Dense(1))
    #model.compile(optimizer='rmsprop', loss='mse', metrics=['mae'])
    #model.compile(optimizer=rmsprop, loss='mse', metrics=['mae'])
    model.compile(optimizer=sgd, loss='mse', metrics=['mae'])
    model.fit(X[train], Y[train], epochs=10, verbose=1)
###########################
y_pred = model.predict(X[test])
print y_pred.round()[:5]
print Y[test][:5]

####################
mse_value, mae_value = model.evaluate(X[test], Y[test], verbose=0)
print(mse_value)
print(mae_value)
print r2_score(Y[test], y_pred)

##########################
# Confusion matrix
#print confusion_matrix(y_test, y_pred.round())

# Precision 
#print precision_score(y_test, y_pred.round())

# Recall
#print recall_score(y_test, y_pred.round())

# F1 score
#print f1_score(y_test,y_pred.round())

# Cohen's kappa
#print cohen_kappa_score(y_test, y_pred.round())

