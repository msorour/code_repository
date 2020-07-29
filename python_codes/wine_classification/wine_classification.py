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

# Print info on white wine
#print(white.info())

# Print info on red wine
#print(red.info())

# First rows of `red` 
#print "red head"
#print red.head()

# Last rows of `white`
#print "white tail"
#print white.tail()

# Take a sample of 5 rows of `red`
#print "red sample"
#print red.sample(5)

#print "white describe"
# Describe `white`
#print white.describe()

#print "red is there null ?"
# Double check for null values in `red`
#print pd.isnull(red)



############################
#fig, ax = plt.subplots(1, 2)

#ax[0].hist(red.alcohol, 10, facecolor='red', alpha=0.5, label="Red wine")
#ax[1].hist(white.alcohol, 10, facecolor='white', ec="black", lw=0.5, alpha=0.5, label="White wine")

#fig.subplots_adjust(left=0.1, right=0.95, bottom=0.15, top=0.7, hspace=0.1, wspace=0.35)
#ax[0].set_ylim([0, 1000])
#ax[0].set_xlabel("Alcohol in % Vol")
#ax[0].set_ylabel("Frequency")
#ax[1].set_xlabel("Alcohol in % Vol")
#ax[1].set_ylabel("Frequency")
#ax[0].legend(loc='best')
#ax[1].legend(loc='best')
#fig.suptitle("Distribution of Alcohol in % Vol")

#print(np.histogram(red.alcohol, bins=[7,8,9,10,11,12,13,14,15]))
#print(np.histogram(white.alcohol, bins=[7,8,9,10,11,12,13,14,15]))
#plt.show()



#########################
#fig, ax = plt.subplots(1, 2, figsize=(8, 4))

#ax[0].scatter(red['quality'], red["sulphates"], color="red")
#ax[1].scatter(white['quality'], white['sulphates'], color="white", edgecolors="black", lw=0.5)

#ax[0].set_title("Red Wine")
#ax[1].set_title("White Wine")
#ax[0].set_xlabel("Quality")
#ax[1].set_xlabel("Quality")
#ax[0].set_ylabel("Sulphates")
#ax[1].set_ylabel("Sulphates")
#ax[0].set_xlim([0,10])
#ax[1].set_xlim([0,10])
#ax[0].set_ylim([0,2.5])
#ax[1].set_ylim([0,2.5])
#fig.subplots_adjust(wspace=0.5)
#fig.suptitle("Wine Quality by Amount of Sulphates")

#np.random.seed(570)

#redlabels = np.unique(red['quality'])
#whitelabels = np.unique(white['quality'])

#fig, ax = plt.subplots(1, 2, figsize=(8, 4))
#redcolors = np.random.rand(6,4)
#whitecolors = np.append(redcolors, np.random.rand(1,4), axis=0)

#for i in range(len(redcolors)):
#    redy = red['alcohol'][red.quality == redlabels[i]]
#    redx = red['volatile acidity'][red.quality == redlabels[i]]
#    ax[0].scatter(redx, redy, c=redcolors[i])
#for i in range(len(whitecolors)):
#    whitey = white['alcohol'][white.quality == whitelabels[i]]
#    whitex = white['volatile acidity'][white.quality == whitelabels[i]]
#    ax[1].scatter(whitex, whitey, c=whitecolors[i])
    
#ax[0].set_title("Red Wine")
#ax[1].set_title("White Wine")
#ax[0].set_xlim([0,1.7])
#ax[1].set_xlim([0,1.7])
#ax[0].set_ylim([5,15.5])
#ax[1].set_ylim([5,15.5])
#ax[0].set_xlabel("Volatile Acidity")
#ax[0].set_ylabel("Alcohol")
#ax[1].set_xlabel("Volatile Acidity")
#ax[1].set_ylabel("Alcohol") 
##ax[0].legend(redlabels, loc='best', bbox_to_anchor=(1.3, 1))
#ax[1].legend(whitelabels, loc='best', bbox_to_anchor=(1.3, 1))
##fig.suptitle("Alcohol - Volatile Acidity")
#fig.subplots_adjust(top=0.85, wspace=0.7)

#plt.show()







#######################
# Add `type` column to `red` with value 1
red['type'] = 1
# Add `type` column to `white` with value 0
white['type'] = 0
# Append `white` to `red`
wines = red.append(white, ignore_index=True)



####################
# Correlation Matrix: correlation between data
#corr = wines.corr()
#sns.heatmap(corr, 
#            xticklabels=corr.columns.values,
#            yticklabels=corr.columns.values)
#plt.show()


#####################
# Specify the data 
X=wines.ix[:,0:11]

# Specify the target labels and flatten the array 
y=np.ravel(wines.type)
#y=np.ravel(wines.quality)

# Split the data up in train and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.33, random_state=42)


## Sample scaling
# Define the scaler 
scaler = StandardScaler().fit(X_train)

# Scale the train set
X_train = scaler.transform(X_train)

# Scale the test set
X_test = scaler.transform(X_test)



#####################
# Initialize the constructor
model = Sequential()

# Add an input layer 
model.add(Dense(12, activation='relu', input_shape=(11,)))

# Add one hidden layer 
model.add(Dense(5, activation='relu'))

# Add an output layer 
model.add(Dense(1, activation='sigmoid'))

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

