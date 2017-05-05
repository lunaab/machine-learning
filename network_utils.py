from keras import backend as K
import tensorflow as tf

"""
network_utils contains functions that can be used
by models regardless of instantiation.
"""


"""
mse_ignore_nan is a mean square error loss function
that ignores nan values from sensor data.

Author: Dr. Nathan Sprague
"""
def mse_ignore_nan(y_true, y_pred):
     ok_entries = tf.logical_not(tf.is_nan(y_true))
     safe_targets = tf.where(ok_entries, y_true, y_pred)
     sqr = tf.square(y_pred - safe_targets)

     zero_nans = tf.cast(ok_entries, K.floatx())
     num_ok = tf.reduce_sum(zero_nans, axis=-1) # count OK entries
     num_ok = tf.maximum(num_ok, tf.ones_like(num_ok)) # avoid divide by zero
     return tf.reduce_sum(sqr, axis=-1) / num_ok

