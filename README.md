SO_DATA MANUAL
===============

The package so_data provides components which are required to implement self-organization. It provides functionality to send data to the soData topic as well as to aggregate, store and use it for 
calculations. Unit tests were done for the core components to ensure correctness of mathematical computations.  

Package components
-------------------

The package consists of the following components:

* **calc.py**: provides helper functions which are used by several components of the package (+ unit test)
* **flocking.py**: provides methods to calculate the flocking vector based on the paper of Olfati-Saber (+ unit test) 
* **soBroadcaster.py**: allows to publish data to the soData topic which will be subscribed to in the soBuffer 
* **soBuffer.py**: implements a layer which allows to store gradient data and to do calculations necessary for self-organizing behaviour (+ unit test)  
* **gradientSensor.py**: implements a complex version of the Simple Topic Sensor, the Gradient Sensor
* **soMessage.msg**: message file describing gradients 


soMessage
---------

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



soBuffer(.py)
-------------

The central component of the so_data package is the soBuffer. It subscribes to the soData topic and stores all received gradients in an aggregated manner. Furthermore, it includes several calculation options 
which are based on gradients. Behaviours can use the soBuffer to implement self-organizing behaviour. The Gradient Sensor uses an instance of the soBuffer too, to enable activation calculation based on 
gradient attraction/repulsion. Various parameters allow to adjust the soBuffer to the needs of a self-organization pattern. 

### soBuffer Parameters 

The soBuffer provides several parameters which can be set to adjust the data storage and calculation behaviour. The following parameters can be set (default values specified in brackets):

* **aggregation** ({'DEFAULT': 'max'}): dictionary (key: frameID, value: aggregation option) of aggregation type per frameID. 'DEFAULT' is used for all frameIDs which have no option specified in 
the dictionary and has to be specified. Does not affect storage of neighbor gradients. Aggregation options:
  * **min** = keep gradient with minimum diffusion radius (at a position / within aggregation_distance)
  * **max** = keep gradient with maximum diffusion radius (at a position / within aggregation_distance)
  * **avg** = keep average gradient (at a position / within aggregation_distance) 
  * **newest** = keep newest / last received gradient (at a position / within aggregation_distance) 
* **min_diffusion** (0.1): float; threshold value specifying minimum diffusion radius gradient has to have when goal_radius == 0.0 to be stored in soBuffer
* **view_distance** (2.0): float; radius in which agent can sense gradients. Should be >= goal_radius specified in agent's own gradient.  
* **id** (''): 'robotX' with X = robot's id; frameID's in this form are considered as robot position data / neighbor gradients. Gradients with own id are stored in self._ownpos 
* **result** (''): specifies how soBuffer data (within agent's view) is aggregated s.t. one value is returned. Options:
  * **all** = movement vector considering all vectors of the potential field will be returned 
  * **max** = movement vector based on maximum repulsion / attraction (goal+diffusion) will be returned 
  * **near** = movement vector following nearest attractive gradient by avoiding repulsive gradients will be returned; robot might not reach gradient source  
  * **reach** = movement vector following nearest attractive gradient by avoiding repulsive gradients will be returned; allows to reach gradient source in comparison to 'near' 
  * **avoid** = movement vector leading away from all sensed gradients will be returned 
* **collision_avoidance** ('repulsion'): collision avoidance between neighbors / agents. Options:
  * **gradient** = gradient/potential field approach is used to calculate repulsion vector (formulas of 'reach' option of result)
  * **repulsion** = repulsion vector is calculated based on formula presented in Fernandez-Marquez et al.
* **store_neighbors** (True): bool; defines whether neighbor gradients / robot position data should be stored 
* **neighbor_storage_size** (2): int; defines number of gradients which will be stored for robot position data / neighbor gradients. 
* **framestorage** ([]): array listing all frameIDs which should be stored. Empty array leads to storage of all frameIDs.
* **aggregation_distance** (1.0): radius in which gradient data is aggregated (see aggregation) 


### data Storage

- neighbors stored separately (frame ID robotX) 


### Methods for use in behaviours and sensors 

The following methods can be used to calculate information necessary for behaviours and sensors. 

* **Agent Density Function**: returns True / False based on the quantity of agents within view distance is over / below threshold. Depends solely on agent gradients (frameID: 'robotX'). 
Can be used to implement Quorum Sensing. Parameter: `threshold` which has to be reached 

```python
def quorum(self, threshold)
```

* **Flocking**: 

```python
def 
```

* **Goal Achievement**: returns True / False based on whether the gradient source was reached or not ('normalized' attraction == 0). E.g. to be used in sensors to bind activation on achievement of goal (e.g. with Boolean Activator). 
 Parameters: current `pose` of the agent, `frameids` specifying which gradients should be considered in the calculation (optional)

```python
def get_goal_reached(self, pose, frameids=[])
```


### Gradient calculation 

The soBuffer includes two different approaches of calculating the attraction/repulsion of gradients. The first approach follows the approach presented in "Social potentials for scalable multi-robot formations"
by Balch and Hybinette (2000). The second approach is based on "New Potential Functions for Mobile Robot Path Planning" by Ge and Cui (1999) which was enhanced with an inner goal_radius which leads to 
infinite repulsion. 

* **Balch and Hybinette (2000)**

The paper of Balch and Hybinette includes formulas to calculate attraction and repulsion of gradients. Attraction values are within `[0,1]` while repulsion values are within `[0, inf.]`. Attraction and 
   repulsion can be combined to generate the movement vector (see [TODO]). In some scenarious the attractive gradient might not be reached as it attraction and repulsion lead to a zero potential value
   at a point not being the attractive gradient source. 
   Parameters: `gradient` is a soMessage with the attractive/repulsive gradient data, `pose` is the current position of the agent. 

```python
def _calc_attractive_gradient(gradient, pose)

def _calc_repulsive_gradient(gradient, pose)
```

* **Ge and Cui (1999)** 

In comparison to the approach by Balch and Hybinette, Ge and Cui guarantee with their approach, that the attractive gradient source is reached. The implementation of the attractive gradient is similar to
Balch and Hybinette, the only difference is that Balch and Hybinette return normalized gradient values while Ge and Cui return absolute values. Therewith, to determine the closest attractive gradient 
the method based on Balch and Hybinette can be used. Ge and Cui enhanced the repulsive gradient calculation to ensure that the gradient source can be reached. 


```python 
def _calc_attractive_gradient_ge(gradient, pose)

def _calc_repulsive_gradient_ge(gradient, goal, pose)
```




flocking(.py)
-------------

The flocking.py file contains algorithms to realize flocking in free-sprace (free flocking). 

1. gradient-based term
2. velocity consensus term 
