#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>


RosThread::RosThread()
{
    shutdown = false;

    startMission = false;

    currentState = HS_STOP;

}



void RosThread::work()
{

    if(!ros::ok()){

        emit this->rosFinished();

        return;
    }

    emit rosStarted();

    ros::Rate loop(10);

    QString path = QDir::homePath();
    path.append("/ISL_workspace/src/configISL.json");


    if(!readConfigFile(path)){

        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        emit rosFinished();
    }



    messageNewTaskInfoSub = n.subscribe("taskObserverISLH/newTaskInfo",queueSize,&RosThread::handleNewTaskMessage, this);

    messageTaskInfo2LeaderPub = n.advertise<ISLH_msgs::taskInfo2LeaderMessage>("taskHandlerISLH/taskInfo2Leader",queueSize);

    messageCmdFromLeaderSub = n.subscribe("messageDecoderISLH/cmdFromLeader",queueSize,&RosThread::handleLeaderCmdMessage, this);

    messageTaskObserveOKPub = n.advertise<std_msgs::UInt8>("taskHandlerISLH/taskObserveOK", queueSize);

    messageTargetPosePub = n.advertise<geometry_msgs::Pose2D>("taskHandlerISLH/targetPose", queueSize);

    messageCurrentPoseSub = n.subscribe("navigationISLH/currentPose", queueSize,&RosThread::handleCurrentPoseMessage, this);

    messageTargetReachedSub = n.subscribe("navigationISLH/targetReached", queueSize, &RosThread::handleTargetReachedMessage, this);

    messageNavigationOKPub = n.advertise<std_msgs::UInt8>("taskHandlerISLH/navigationOK", queueSize);

    taskHandlerStatePub = n.advertise<std_msgs::UInt8>("taskHandlerISLH/robotInfo2Monitor", queueSize);

    while(ros::ok())
    {

        this->manageTaskHandling();

        ros::spinOnce();

        loop.sleep();

    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}

void RosThread::manageTaskHandling()
{

    // new task was detected
    if (newTasksList.isEmpty()==false)
    {
        // if the robot (coalition) is in idle state,
        if (currentState == HS_IDLE)
        {
            std_msgs::UInt8 msgTaskObserveOK;
            msgTaskObserveOK.data = 0;

            // stop observing task while trying to handle a task
            messageTaskObserveOKPub.publish(msgTaskObserveOK);

            // the robot stays in its position while waiting for an answer from the leader
            geometry_msgs::Pose2D msgTagetPose;

            msgTagetPose.x = currentPose.X;
            msgTagetPose.y = currentPose.Y;

            messageTargetPosePub.publish(msgTagetPose);

            currentState = HS_WAITING_TASK_RESPONSE_FROM_LEADER;

            std_msgs::UInt8 msg;
            msg.data = currentState;
            taskHandlerStatePub.publish(msg);
        }

        ISLH_msgs::taskInfo2LeaderMessage msg;

        msg.infoMessageType = INFO_R2L_NEW_TASK_INFO;
        msg.taskUUID = newTasksList.at(0).taskUUID.toStdString();
        msg.posX = newTasksList.at(0).pose.X;
        msg.posY = newTasksList.at(0).pose.Y;
        msg.senderRobotID = ownRobotID;
        msg.receiverRobotID = leaderRobotID;
        msg.handlingDuration = newTasksList.at(0).handlingDuration;
        msg.timeOutDuration = newTasksList.at(0).timeOutDuration;
        msg.requiredResources = newTasksList.at(0).requiredResources.toStdString();
        msg.encounteringTime = newTasksList.at(0).encounteringTime;

        // report this new task to the coalition leader
        messageTaskInfo2LeaderPub.publish(msg);

        // update waiting task time info
        waitingTask.encounteringTime = newTasksList.at(0).encounteringTime;
        waitingTask.timeOutDuration = newTasksList.at(0).timeOutDuration;

        // remove this newly incoming task from  newTaskList vector
        newTasksList.remove(0);
    }


    if (currentState == HS_STOP)
    {
        if (startMission == true)
        {
            currentState = HS_IDLE;

            std_msgs::UInt8 msg;
            msg.data = currentState;
            taskHandlerStatePub.publish(msg);
        }

    }
    else if (currentState == HS_HANDLING)
    {
        uint currentTime = QDateTime::currentDateTime().toTime_t();

        if ( (currentTime-handlingTask.startHandlingTime)>=handlingTask.handlingDuration )
        {
            completedTasks.append(handlingTask);

            currentState = HS_IDLE;

            std_msgs::UInt8 msg;
            msg.data = currentState;
            taskHandlerStatePub.publish(msg);

            std_msgs::UInt8 msgTaskObserveOK;
            msgTaskObserveOK.data = 1;
            // start observing task
            messageTaskObserveOKPub.publish(msgTaskObserveOK);
        }
    }

    // waiting for a response from the leader
    else if (currentState == HS_WAITING_TASK_RESPONSE_FROM_LEADER)
    {
        // if task timeout then continue observing
        uint currentTime = QDateTime::currentDateTime().toTime_t();
        if (currentTime - waitingTask.encounteringTime >= waitingTask.timeOutDuration)
        {
            currentState = HS_IDLE;

            std_msgs::UInt8 msg;
            msg.data = currentState;
            taskHandlerStatePub.publish(msg);

            // start observing task
            std_msgs::UInt8 msgTaskObserveOK;
            msgTaskObserveOK.data = 1;
            messageTaskObserveOKPub.publish(msgTaskObserveOK);
        }
    }

}



void RosThread::handleNewTaskMessage(ISLH_msgs::newTaskInfoMessage msg)
{

    taskProp newTask;

    std::time_t arrivalTime = std::time(0);
    qDebug()<< " Task - timestamp " << msg.timeStamp << " arrival time " << arrivalTime;

    newTask.taskUUID = QString::fromStdString(msg.taskUUID);
    newTask.handlingDuration = msg.handlingDuration;
    newTask.timeOutDuration = msg.timeOutDuration;

    newTask.requiredResources = QString::fromStdString(msg.requiredResources);

    newTask.pose.X = currentPose.X;
    newTask.pose.Y = currentPose.Y;

    newTask.startHandlingTime = -1;

    newTask.status = 0;

    newTask.encounteringTime = msg.timeStamp;

    newTasksList.append(newTask);

}

void RosThread::handleLeaderCmdMessage(ISLH_msgs::cmdFromLeaderMessage msg)
{
    if (msg.cmdTypeID == CMD_L2R_START_OR_STOP_MISSION)
    {
        QString msgStr = QString::fromStdString(msg.cmdMessage);

        if (msgStr == "START-MISSION")
        {
            startMission = true;

            // start observing task while trying to handle a task
            std_msgs::UInt8 msgTaskObserveOK;
            msgTaskObserveOK.data = 1;
            messageTaskObserveOKPub.publish(msgTaskObserveOK);

            // start navigation
            std_msgs::UInt8 msgNavigationOK;
            msgNavigationOK.data = 1;
            messageNavigationOKPub.publish(msgNavigationOK);

        }
        else if (msgStr == "STOP-MISSION")
        {
            startMission = false;

            // stop observing task while trying to handle a task
            std_msgs::UInt8 msgTaskObserveOK;
            msgTaskObserveOK.data = 0;
            messageTaskObserveOKPub.publish(msgTaskObserveOK);

            // stop navigation
            std_msgs::UInt8 msgNavigationOK;
            msgNavigationOK.data = 0;
            messageNavigationOKPub.publish(msgNavigationOK);


            currentState = HS_STOP;

            std_msgs::UInt8 msg;
            msg.data = currentState;
            taskHandlerStatePub.publish(msg);
        }
    }
    else if (msg.cmdTypeID==CMD_L2R_MOVE_TO_TASK_SITE)
    {
        // message content: taskSitePoseX, taskSitePoseY

        QString cmdMessage = QString::fromStdString(msg.cmdMessage);

        qDebug() << "CMD_L2R_MOVE_TO_TASK_SITE -> msg.cmdMessage: "<<cmdMessage;

        // Split the data (Comma seperated format)
        QStringList cmdList = cmdMessage.split(",",QString::SkipEmptyParts);

        // Go to the task site defined by the leader
        geometry_msgs::Pose2D msgTagetPose;

        msgTagetPose.x = cmdList.at(0).toDouble();
        msgTagetPose.y = cmdList.at(1).toDouble();

        messageTargetPosePub.publish(msgTagetPose);

        currentState = HS_SUCCORING;

        std_msgs::UInt8 msg;
        msg.data = currentState;
        taskHandlerStatePub.publish(msg);

    }
    else if  (msg.cmdTypeID==CMD_L2R_START_HANDLING)
    {
        // message content: taskUUID, handlingDuration

        QString cmdMessage = QString::fromStdString(msg.cmdMessage);
        // Split the data (Comma seperated format)
        QStringList cmdList = cmdMessage.split(",",QString::SkipEmptyParts);

        handlingTask.taskUUID = cmdList.at(0);

        handlingTask.handlingDuration = cmdList.at(1).toUInt();

        handlingTask.pose.X = currentPose.X;
        handlingTask.pose.Y = currentPose.Y;

        handlingTask.startHandlingTime = QDateTime::currentDateTime().toTime_t();

        handlingTask.status = 1;

        // stop the robot by setting current pose as target pose since it is within the task site
        geometry_msgs::Pose2D msgTagetPose;
        msgTagetPose.x = currentPose.X;
        msgTagetPose.y = currentPose.Y;
        messageTargetPosePub.publish(msgTagetPose);

        currentState = HS_HANDLING;

        std_msgs::UInt8 msg;
        msg.data = currentState;
        taskHandlerStatePub.publish(msg);

    }
    else if  (msg.cmdTypeID==CMD_L2R_MOVE_TO_GOAL_POSE)
    {
        // message content: goalPoseX, goalPoseY

        QString cmdMessage = QString::fromStdString(msg.cmdMessage);
        // Split the data (Comma seperated format)
        QStringList cmdList = cmdMessage.split(",",QString::SkipEmptyParts);

        // Go to the goal pose
        geometry_msgs::Pose2D msgTagetPose;
        msgTagetPose.x = cmdList.at(0).toDouble();
        msgTagetPose.y = cmdList.at(1).toDouble();
        messageTargetPosePub.publish(msgTagetPose);

        currentState = HS_IDLE;

        std_msgs::UInt8 msg;
        msg.data = currentState;
        taskHandlerStatePub.publish(msg);
    }
    else if (msg.cmdTypeID == CMD_L2R_SPLIT_FROM_COALITION)
    {
        // the robot was removed from the coalition,
        // hence it now is leader of itself
        leaderRobotID = ownRobotID;

        // since robot changed coalition current state must be idle
        // and we must continue observing task
        currentState = HS_IDLE;

        std_msgs::UInt8 msg;
        msg.data = currentState;
        taskHandlerStatePub.publish(msg);

        // start observing task
        std_msgs::UInt8 msgTaskObserveOK;
        msgTaskObserveOK.data = 1;
        messageTaskObserveOKPub.publish(msgTaskObserveOK);


        qDebug()<< " Splitted from the coalition";
    }
    else if ( (msg.cmdTypeID == CMD_L2R_LEADER_CHANGED) || (msg.cmdTypeID == CMD_L2R_I_AM_LEADER) )
    {
        QString cmdMessage = QString::fromStdString(msg.cmdMessage);

        cmdMessage.remove("NewLeaderID");

        leaderRobotID = cmdMessage.toInt();

        qDebug()<< " Leader was changed. New leader ID: " << leaderRobotID;
    }
}

//Update currentPose with the pose info coming from navigationISLH
void RosThread::handleCurrentPoseMessage(geometry_msgs::Pose2D msg)
{
    currentPose.X = msg.x;
    currentPose.Y = msg.y;
}

void RosThread::handleTargetReachedMessage(std_msgs::UInt8 msg)
{
    if (msg.data==1) // if target position is reached
    {
        if (currentState == HS_SUCCORING) // while succoring (moving to task site)
        {
            ISLH_msgs::taskInfo2LeaderMessage msg;

            msg.infoMessageType = INFO_R2L_REACHED_TO_TASK;
            msg.senderRobotID = ownRobotID;
            msg.receiverRobotID = leaderRobotID;
            msg.reachingTime = QDateTime::currentDateTime().toTime_t();

            // report this to the coalition leader
            messageTaskInfo2LeaderPub.publish(msg);

            currentState = HS_WAITING_HANDLE_START_FROM_LEADER;

            std_msgs::UInt8 _msg;
            _msg.data = currentState;
            taskHandlerStatePub.publish(_msg);
        }
        else if (currentState == HS_IDLE) // while moving to goal pose (detecting task)
        {
            ISLH_msgs::taskInfo2LeaderMessage msg;

            msg.infoMessageType = INFO_R2L_REACHED_TO_GOAL;
            msg.senderRobotID = ownRobotID;
            msg.receiverRobotID = leaderRobotID;
            msg.reachingTime = QDateTime::currentDateTime().toTime_t();

            // report this to the coalition leader
            messageTaskInfo2LeaderPub.publish(msg);

            currentState = HS_WAITING_GOAL_POSE_FROM_LEADER;

            std_msgs::UInt8 _msg;
            _msg.data = currentState;
            taskHandlerStatePub.publish(_msg);
        }
    }
}

// Reads the config file
bool RosThread::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {
        ownRobotID = result["robotID"].toInt();

        leaderRobotID = ownRobotID;


        queueSize = result["queueSize"].toInt();
        qDebug()<<result["queueSize"].toString();

    }
    file.close();
    return true;

}
