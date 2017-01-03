# SO_DATA MANUAL

The package so_data provides components which are required to implement self-organization. It provides functionality to send data to the soData topic as well as to aggregate, store and use it for 
calculations. Unit tests were done for the core components to ensure correctness of mathematical computations.  

## Package components

The package consists of the following components:

* **calc.py**: provides helper functions which are used by several components of the package (+ unit test)
* **flocking.py**: provides methods to calculate the flocking vector based on the paper of Olfati-Saber (+ unit test) 
* **soBroadcaster.py**: allows to publish data to the soData topic which will be subscribed to in the soBuffer 
* **soBuffer.py**: implements a layer which allows to store gradient data and to do calculations necessary for self-organizing behaviour (+ unit test)  
* **gradientSensor.py**: implements a complex version of the Simple Topic Sensor, the Gradient Sensor
* **soMessage.msg**: message file describing gradients 


## soMessage

elements:

* **Header header**
  * time stamp 
  * frameID
  * sequence number
* 
*

reserved frameIDs:
* **'robot' + ID**: frameID indicating that gradient of another robot is received
* **'DEFAULT'**: value is used to specify aggregation option (store_data in soBuffer) for frameIDs having no specific option assigned to 
* **'None'**: 



## soBuffer

The central component of the so_data package is the soBuffer. It subscribes to the soData topic and stores all received gradients in an aggregated manner. Furthermore, it includes several calculation options 
which are based on gradients. Behaviours can use the soBuffer to implement self-organizing behaviour. The Gradient Sensor uses an instance of the soBuffer too, to enable activation calculation based on 
gradient attraction/repulsion. Various parameters allow to adjust the soBuffer to the needs of a self-organization pattern. 

### soBuffer Parameters 

The soBuffer provides several parameters which can be set to adjust the data storage and calculation behaviour. The following parameters can be set:

* **aggregation**:


### data Storage

- neighbors stored separately (frame ID robot bla) 

- the parameter epsilon remains fixed throughout the paper, epsilon > 0 
- parameter epsilon within (0, 1) 

### methods 
gradients etc. 


## flocking

Algorithms for flocking in free-space (free flocking).  

1. gradient-based term
2. velocity consensus term 



```python
computeActivation() 
computeSatisfaction()
self.computeWishes()
self.getProgress()
```