Waiting for receives a True message from the Navigation stack. Once it receive, begin to subscribe to the /target_position information published by YOLO Object Detection to confirm the location of the target object. 
Using mathematical algorithm to calculates the relative position of the object and the End Effector.
Publishes the /target_position to the End Effector. 
Subscribe to End effector /end_effector(Boolean)
