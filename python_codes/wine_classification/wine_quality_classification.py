import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from keras.models import Sequential
from keras.layers import Dense
from sklearn.metrics import confusion_matrix, precision_score, recall_score, f1_score, cohen_kappa_score


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
y = wines.quality

# Isolate data
X = wines.drop('quality', axis=1)

#############################
# Split the data up in train and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.33, random_state=42)


## Sample scaling
# Define the scaler 
#scaler = StandardScaler().fit(X_train)

# Scale the train set
#X_train = scaler.transform(X_train)

# Scale the test set
#X_test = scaler.transform(X_test)

# Scale the data with `StandardScaler`
X = StandardScaler().fit_transform(X)

#####################
# Initialize the constructor
model = Sequential()

# Add an input layer 
#model.add(Dense(12, activation='relu', input_shape=(11,)))

# Add one hidden layer 
#model.add(Dense(5, activation='relu'))

# Add an output layer 
#model.add(Dense(1, activation='sigmoid'))


# Add input layer 
model.add(Dense(64, input_dim=12, activation='relu'))
    
# Add output layer 
model.add(Dense(1))

####################
# Model output shape
model.output_shape

# Model summary
model.summary()

# Model config
model.get_config()

# List all weight tensors 
model.get_weights()


######################
model.compile(loss='binary_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])
                   
model.fit(X_train, y_train,epochs=20, batch_size=1, verbose=1)


#########################
score = model.evaluate(X_test, y_test,verbose=1)

print(score)

###########################
y_pred = model.predict(X_test)
print y_pred.round()[:5]
print y_test[:5]

##########################
# Confusion matrix
print confusion_matrix(y_test, y_pred.round())

# Precision 
print precision_score(y_test, y_pred.round())

# Recall
print recall_score(y_test, y_pred.round())

# F1 score
print f1_score(y_test,y_pred.round())

# Cohen's kappa
print cohen_kappa_score(y_test, y_pred.round())

