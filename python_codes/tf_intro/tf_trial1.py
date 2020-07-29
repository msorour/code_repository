import tensorflow as tf
import numpy as np

#activate a tensorflow interactive session
tf.InteractiveSession()

#define tensors
a = tf.zeros((2,2))
b = tf.ones((2,2))

#sum the elements of the matrix (2D tensor) across the horizontal axis
print(tf.reduce_sum(b,reduction_indices=1).eval())

rw=tf.Variable(tf.random_normal((2,2)), name='random_weights')

with tf.Session() as sess:
  sess.run(tf.global_variables_initializer())
  print(sess.run(rw))
