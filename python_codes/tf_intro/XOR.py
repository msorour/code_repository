import tensorflow as tf

# PREPARATION
# create placeholders for training input and output labels
x_ = tf.placeholder(tf.float32,shape=[4,2],name="x-input")
y_ = tf.placeholder(tf.float32,shape=[4,1],name="y-input")

# define weights to the hidden and output layers
w1 = tf.Variable(tf.random_uniform([2,2],-1,1), name="weights1")
w2 = tf.Variable(tf.random_uniform([2,1],-1,1), name="weights2")

# define the bias for hidden and output layers
b1 = tf.Variable(tf.zeros([2]),name="bias1")
b2 = tf.Variable(tf.zeros([1]),name="bias2")

# define final output through forward pass
z2 = tf.sigmoid(tf.matmul(x_,w1)+b1)
prediction = tf.sigmoid(tf.matmul(z2,w2)+b2)

# define cross-entropy/log-loss cost function based on the output label y and the predicted probability in forward pass
cost = tf.reduce_mean(( (y_ *tf.log(prediction)) + (1-y_)*tf.log(1.0-prediction) ) * -1)
learning_rate = 0.01
#train_step = tf.train.GradientDescentOptimizer(learning_rate).minimize(cost)
train_step = tf.train.AdamOptimizer(learning_rate).minimize(cost)
#train_step = tf.train.MomentumOptimizer(learning_rate,momentum=0.9).minimize(cost)



# START TRAINING
# training data
XOR_X = [[0,0],[0,1],[1,0],[1,1]]
XOR_Y = [[0],[1],[1],[0]]

# initialize variables
#init = tf.initialize_all_variables()
init = tf.global_variables_initializer()
sess = tf.Session()
writer = tf.summary.FileWriter("./logs/XOR_logs", sess.graph_def)

sess.run(init)
for i in range(1000):
    sess.run(train_step, feed_dict={x_:XOR_X, y_:XOR_Y})

print('final prediction', sess.run(prediction, feed_dict={x_:XOR_X, y_:XOR_Y}))
