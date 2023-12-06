When Manipulator receives a True message from the Navigation stack, it subscribes to the /target_position information published by YOLO Object Detection to confirm the location of the target object. 
Using mathematical algorithm to calculates the relative position of the object and the End Effector.
publishes the updated object position information to the End Effector. 
