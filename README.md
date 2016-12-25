This thread is used for updating my modelling codes with reference to path planning. 
The following path planning algorithm are desired to be updated
A* - star
RRT Algorithm
Automated exploring 2D mobot using sensored control against odometric control and validation of their accuracy.
A* - star
  This algorithm moves to the target position from home position through finding the nearest neighbour where the choice of neighbour is based on distance from the target
  To avoid confustion, a floor map is developed and based on the floor map, the distance is multiplied n (n = nx*ny) times to reduce the confusion.
  Now, rigid body blocks are arranged on their path through initializing infinity blocks in the room map. However for ease of operation, instead of infinity, the cells are numbered as (nx*ny) + floor map to keep the cell out of count
  
