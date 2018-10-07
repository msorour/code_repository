import tensorflow as tf
import numpy as np
from sklearn.datasets import load_boston
import matplotlib.pyplot as plt

# PREPARATION
# function to load Boston dataset
def read_infile():
    data = load_boston()
    features = np.array(data.data)
    target = np.array(data.target)
    return features, target

# normalize features: subtract from each feature the mean, then divide by the standard deviation, better for GradientDescent
def feature_normalize(data):
    mean = np.mean(data,axis=0)
    standard_deviation = np.std(data,axis=0)
    return (data-mean)/standard_deviation

# append the bias term to the feature vector
def append_bias(features,target):
    n_samples = features.shape[0]
    n_features = features.shape[1]
    intercept_feature = np.ones((n_samples,1))
    X = np.concatenate((features,intercept_feature),axis=1)
    X = np.reshape(X,[n_samples,n_features+1])
    Y = np.reshape(target,[n_samples,1])
    return X,Y

features,target = read_infile()
z_features = feature_normalize(features)
X_input, Y_input = append_bias(z_features,target)
num_features = X_input.shape[1]

# create placeholders, weights
X = tf.placeholder(tf.float32,[None,num_features])
Y = tf.placeholder(tf.float32,[None,1])
w = tf.Variable(tf.random_uniform((num_features,1)), name="weights")

# define cost and optimization parameters
learning_rate = 0.01
num_epochs = 1000
cost_trace = []
prediction = tf.matmul(X,w)
error = prediction - Y
cost = tf.reduce_mean(tf.square(error))
train_op = tf.train.GradientDescentOptimizer(learning_rate).minimize(cost)
#train_op = tf.train.AdamOptimizer(learning_rate).minimize(cost)
#train_op = tf.train.MomentumOptimizer(learning_rate,momentum=0.9).minimize(cost)




# START TRAINING
init = tf.global_variables_initializer()
sess = tf.Session()
sess.run(init)
for i in xrange(num_epochs):
    sess.run(train_op, feed_dict={X:X_input, Y:Y_input})
    cost_trace.append(sess.run(cost, feed_dict={X:X_input, Y:Y_input}))
error_ = sess.run(error, feed_dict={X:X_input, Y:Y_input})
prediction_ = sess.run(prediction, feed_dict={X:X_input})

print 'MSE in training: ', cost_trace[-1]

# plotting cost evolution
plt.plot(cost_trace)
plt.show()

# plotting predicted vs actual house prices
fig, ax = plt.subplots()
plt.scatter(Y_input,prediction_)
ax.set_xlabel('Actual Prices')
ax.set_ylabel('Predicted Prices')
plt.show()