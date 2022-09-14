

###### discussions regarding linear velocity and angular velocity
rigid body velocities - twist
- linear 
- angular
always w.r.t to a frame
```
v_w = w x r
p^dot = v_body + v_w
S - skew symmetric matrix
a x p = S(a).p = (S(p)^T).a
```
angular velocities remain **same** for every point on the rotating body

---
###### function and use of grasp matrix and hand jacobian

They define relationship between 
Grasp matrix - _object velocity_ and _contact velocities_
Hand Jacobian -  _joint angles and contact velocities_ 

Ultimately giving us relationship between joint angles and object velocity
so control algorithm has to focus on getting the joint angles to required values so that object can be moved that way

---
 
###### discussions regarding grasp taxonomies 
-  and their uses for such categories

_power_ and _precision_ are main types
* within power prehensile and non prehensile which doesnt make sense

most widely used **feix taxonomy**
* has power, intermediate and precision categories
* within them thumb adducted and thumb abducted 
	* basically thumb closed with fingers or straight up

###### Uses of grasping taxonomies
task can be programtically mapped the required grasp
control algorithms can be more focused 
- generalization for different grasps is advantage
grippers can be task specific
- again generalization is better
- 