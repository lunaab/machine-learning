import numpy as np
import network_utils as nu

#Set to true to save model after training
save_model = True

image_dest = np.load('io_net_data/elim_z_image_data.npy')
depth_dest = np.load('io_net_data/elim_z_depth_data.npy')

#here the model architecture is defined
#using TS for background so depth data has to be ordered (width, heighth, depth)
model = nu.define_model(nu.mse_ignore_nan)
#model = nu.define_model('mean_squared_error')

#fit the model to matching depth data
model.fit(image_dest, depth_dest, nb_epoch=10, batch_size=32, validation_split=0.2)

#evaluate the performance of the model
scores = model.evaluate(image_dest, depth_dest)
print("%s: %.2f%%" % ('Performance: ', scores[1]*100))

if save_model:
    model.save("model_depth_nan.h5")
