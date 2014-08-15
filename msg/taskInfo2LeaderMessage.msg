# The task's information to coalition leader

uint64 sendingTime #The sending time

uint16 receiverRobotID #robot id to which the message will be sent 

uint16 senderRobotID #robot id which encounters the task

uint16 infoMessageType

string taskUUID # the task's UUID

uint64 encounteringTime #The encountering time in timestamp format

uint64 reachingTime # The time when the robot reached to the task site

uint16 handlingDuration

uint16 timeOutDuration

string requiredResources # Required resources for this task

float32 posX # The position of the task at x direction 
float32 posY # The position of the task at y direction 
