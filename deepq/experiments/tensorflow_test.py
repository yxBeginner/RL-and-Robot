import tensorflow as tf
import tensorflow.contrib.layers as layers

out = tf.Variable([[1.0, 2.0, 3.0], [2.3, 2.4, 5.6]])
fc = layers.fully_connected(out, num_outputs=10, activation_fn=None)

with tf.Session() as sess:
    tf.global_variables_initializer()
    print(fc)
