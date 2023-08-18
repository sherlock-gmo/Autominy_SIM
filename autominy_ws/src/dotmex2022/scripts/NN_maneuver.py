import numpy as np
from keras.models import Sequential
from keras.layers.core import Dense

# Definimos las senales de entrada en el siguiente orden
# [s_red s_peat s_ev s_est ]
x_train = np.zeros((16,4))
for i in range (0,16):
  x_bin = np.binary_repr(i,4)
  # Convert the binary number to a list of integers
  binary_list = [int(x) for x in list(x_bin)]
  # Convert the list to a NumPy array
  x_train[i] = np.array(binary_list)
#print('Las entradas son: \n')
#print(x_train)

# Definimos las senales de salida en el siguiente orden
# [Alto S.Carril Evasion Estacionamiento]
y_train = np.zeros((16,4))
for j in range (0,16):
  red, peat, ev, est = x_train[j,:]  
  # Condiciones para relacionar a las entradas con las salidas
  if (red==0) and (peat==0) and (ev==0): y_train[j] = np.array([0,1,0,0])
  if (red==0) and (peat==0) and (ev==1): y_train[j] = np.array([0,0,1,0])
  if (red==1) or (peat==1): y_train[j] = np.array([1,0,0,0])
  if (est==1): y_train[j] = np.array([0,0,0,1])
#print('*****************************************')
#print('Las salidas son: \n')
#print(y_train)
#print('*****************************************')
#________________________Construccion de la RN
model = Sequential() 									                
model.add(Dense(5, input_dim=4, activation='relu'))		
model.add(Dense(4, activation='softmax'))				      
model.summary()                                       
#________________________Entrenamiento de la red neuronal
# loss---Es la funcion de costo que se optimiza.
# optimizer---Es el metodo de aprendizaje usado.
# metrics---Es la medicion de la presicion de la RN. Es independiente del entrenamiento.
# epochs---Es el numero de epocas del entrenamiento. Este es el criterio de paro.
# batch---Es el número de ejemplos que se introducen en la red para que entrene en cada epoca. 
#sgd = SGD(lr=0.1,momentum=0.01) # Parametros de aprendizaje
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['binary_accuracy'])
model.fit(x_train, y_train, epochs=1000, batch_size=4)


#________________________Evaluacion del desempeno de la red
# En este caso el conjunto de prueba es el mismo que el de entrenamiento
scores = model.evaluate(x_train, y_train)
print('\n%s: %.2f%%' % (model.metrics_names[1], scores[1]*100))
print('Salida de la red a las entradas \n',x_train)
print (model.predict(x_train).round())
path = '/home/sherlock/behavior_selector_nn/'
model.save_weights(path+'NN_maneuver.hdf5')
#model.save(path+'behavior_selector_NN.h5')


