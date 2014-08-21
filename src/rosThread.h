#include <ros/ros.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QtCore/QString>
#include <taskHandlerISLH/taskInfo2LeaderMessage.h>
#include <messageDecoderISLH/cmdFromLeaderMessage.h>
#include <messageDecoderISLH/taskInfoFromRobotMessage.h>
#include <taskObserverISLH/newTaskInfoMessage.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose2D.h>

enum HandlingState
{
    HS_IDLE = 0,
    HS_WAITING_TASK_RESPONSE_FROM_LEADER = 1,
    HS_WAITING_GOAL_POSE_FROM_LEADER = 2,
    HS_SUCCORING = 3,
    HS_WAITING_HANDLE_START_FROM_LEADER = 4,
    HS_HANDLING = 5
};

enum Leader2RobotCmdMsgs
{
    CMD_L2R_START_HANDLING = 1,
    CMD_L2R_MOVE_TO_TASK_SITE = 2,
    CMD_L2R_MOVE_TO_GOAL_POSE = 3,
    CMD_L2R_SPLIT_FROM_COALITION = 4,
    CMD_L2R_LEADER_CHANGED = 5
};


enum Robot2LeaderInfoMgs
{
    INFO_R2L_NEW_TASK_INFO = 1,
    INFO_R2L_REACHED_TO_TASK = 2,
    INFO_R2L_REACHED_TO_GOAL = 3
};

struct poseXY{
    double X;
    double Y;
};

// task properties
struct taskProp{
  QString taskUUID;
  uint encounteringTime; // in timestamp - "the time when the task is encountered"
  uint responsibleUnit;  // "who is responsible for the task"
  uint encounteringRobotID;  // "Id of the robot encountering the task"
  uint handlingDuration; // in seconds - "the required time to handle the task"
  uint timeOutDuration; // "the timed-out duration for the task"
  int status; // "status of the task"
              //
  uint startHandlingTime; // in timestamp - "the time when the task starts being handled"
  poseXY pose; // the location of the task
  //QVector < double > requiredResources;
  QString requiredResources;
};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();


private:
     bool shutdown;

     ros::NodeHandle n;

     ros::Publisher messageTaskInfo2LeaderPub;

     //ros::Publisher messageTaskInfoFromRobot;

     ros::Publisher messageTaskObserveOKPub;

     ros::Publisher messageTargetPosePub;

     ros::Subscriber messageNewTaskInfoSub;

     ros::Subscriber messageCmdFromLeaderSub;

     ros::Subscriber messageCurrentPoseSub;

     ros::Subscriber messageTargetReachedSub;


     HandlingState currentState;

     poseXY currentPose; // X & Y coordinates

     int ownRobotID;

     int leaderRobotID;

     QVector <taskProp> newTasksList;        
     taskProp waitingTask;
     QVector <taskProp> completedTasks;
     QVector <taskProp> timedoutTasks;

     taskProp handlingTask;

     bool readConfigFile(QString filename);

     void manageTaskHandling();

     void handleNewTaskMessage(taskObserverISLH::newTaskInfoMessage msg);

     void handleLeaderCmdMessage(messageDecoderISLH::cmdFromLeaderMessage msg);

     void handleCurrentPoseMessage(geometry_msgs::Pose2D msg);

     void handleTargetReachedMessage(std_msgs::UInt8 msg);

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void rosStarted();
   void rosStartFailed();

};
