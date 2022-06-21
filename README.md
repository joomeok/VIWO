# A Proposal for Odometry of a Greenhouse Robot
 Algorithm used
+ Visual Odometry - ORB SLAM2     
+ EKF - Robot_localization  
+ Methods of fusing sensors - Proposed algorithm, VO, VIO

Each algorithms was verified in the following scenarios
+ Scenario 1 - On Pipe
+ Scenario 2 - On Headland
+ Sceanrio 3 - Entire enviornment (Validation)  
#
## Config 
4 different config.yaml files for robot_localization package
#
## rosmsg_to_tum
Save various ros message into TUM.txt files
#
## sensor_filter_node
Proposed algorithm used in the research.   

![1655787996122](https://user-images.githubusercontent.com/91611693/174720739-19e52cbb-3012-451e-9086-c3f2e18700e2.png)

## Results
![1](https://user-images.githubusercontent.com/91611693/174722952-ab03069d-d94b-4de2-971e-a3d394d57a10.png)

![2](https://user-images.githubusercontent.com/91611693/174722963-1ee98568-b642-4176-bcdd-59983c74aa96.png)

![3](https://user-images.githubusercontent.com/91611693/174722970-abccb97a-f693-4aa7-b035-f866dc29d669.png)
