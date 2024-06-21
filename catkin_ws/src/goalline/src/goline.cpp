#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sendImageToLine(const std::string &imagePath, const std::string &token) {
    CURL *curl;
    CURLcode res;
    curl_mime *form = NULL;
    curl_mimepart *field = NULL;

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if(curl) {
        form = curl_mime_init(curl);

        field = curl_mime_addpart(form);
        curl_mime_name(field, "message");
        curl_mime_data(field, "Image from robot", CURL_ZERO_TERMINATED);

        field = curl_mime_addpart(form);
        curl_mime_name(field, "imageFile");
        curl_mime_filedata(field, imagePath.c_str());

        curl_easy_setopt(curl, CURLOPT_URL, "https://notify-api.line.me/api/notify");
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, curl_slist_append(NULL, ("Authorization: Bearer " + token).c_str()));

        res = curl_easy_perform(curl);

        if(res != CURLE_OK)
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

        curl_easy_cleanup(curl);
        curl_mime_free(form);
    }
    curl_global_cleanup();
}

void takePhoto(const std::string &filePath) {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        ROS_ERROR("Error opening video stream or file");
        return;
    }

    cv::Mat frame;
    cap >> frame;
    if (!frame.empty()) {
        cv::imwrite(filePath, frame);
    }
    cap.release();
}

int main(int argc, char** argv){ 
    ros::init(argc, argv, "my_navigation_goals");
    ros::NodeHandle n;

    ros::Duration(20).sleep();

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";  //ทางเดินขวา
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.56584632397;
    goal.target_pose.pose.position.y = -12.5456218719;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.00500004331539;
    goal.target_pose.pose.orientation.w = 0.999987499705;

    ROS_INFO("Sending Goal 1");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 1");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 1");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ระเบียง
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 17.4282417297;
    goal.target_pose.pose.position.y = -12.799536705;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.00333260826418;
    goal.target_pose.pose.orientation.w = 0.999994446846;

    ROS_INFO("Sending Goal 2");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 2");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 2");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //บันได
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 17.4282417297;
    goal.target_pose.pose.position.y = -12.799536705;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.717632612199;
    goal.target_pose.pose.orientation.w = 0.696421879257;

    ROS_INFO("Sending Goal 3");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 3");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 3");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ทางเดินขวา
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 17.4282417297;
    goal.target_pose.pose.position.y = -12.799536705;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.999990934254;
    goal.target_pose.pose.orientation.w = -0.0042580992437;

    ROS_INFO("Sending Goal 4");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 4");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 4");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map"; //ห้องไฟ
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 9.21243667603;
    goal.target_pose.pose.position.y = -12.7328014374;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.710633389696;
    goal.target_pose.pose.orientation.w = 0.703562495767;

    ROS_INFO("Sending Goal 5");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 5");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 5");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ห้อง605
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.69019687176;
    goal.target_pose.pose.position.y = -12.657576561;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.999956920806;
    goal.target_pose.pose.orientation.w = -0.00928205432938;

    ROS_INFO("Sending Goal 6");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 6");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 6");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map"; //ทางยาวจากขวา
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.69019687176;
    goal.target_pose.pose.position.y = -12.657576561;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.716134147506;
    goal.target_pose.pose.orientation.w = 0.6979626654747;

    ROS_INFO("Sending Goal 7");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 7");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 7");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ห้องอาจารย์ 1
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.38952851295;
    goal.target_pose.pose.position.y = 0.794316112995;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.937993304449;
    goal.target_pose.pose.orientation.w = 0.346653372706;

    ROS_INFO("Sending Goal 8");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 8");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 8");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //จุดเล็กหน้าเสา 1
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.22000837326;
    goal.target_pose.pose.position.y = 9.90415382385;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.725316184146;
    goal.target_pose.pose.orientation.w = 0.688415886667;

    ROS_INFO("Sending small 1");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 1");
    } else {
        ROS_INFO("The base failed to move to small 1");
    }


    goal.target_pose.header.frame_id = "map";  //ทางเดินยาวกลาง
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.743052840233;
    goal.target_pose.pose.position.y = 26.6039733887;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.722836198069;
    goal.target_pose.pose.orientation.w = 0.691019414171;

    ROS_INFO("Sending Goal 9");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 9");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 9");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ห้องเจ็กต์
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.03868603706;
    goal.target_pose.pose.position.y = 19.9064292908;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.999796512111;
    goal.target_pose.pose.orientation.w = -0.0201726143722;

    ROS_INFO("Sending Goal 10");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 10");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 10");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //จุดเล็กเลยเสา 2
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.819928705692;
    goal.target_pose.pose.position.y = 32.5599212646;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.722836198069;
    goal.target_pose.pose.orientation.w = 0.691019414171;

    ROS_INFO("Sending small 2");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 2");
    } else {
        ROS_INFO("The base failed to move to small 2");
    }


    goal.target_pose.header.frame_id = "map";  //ห้องอาจารย์ 2
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.13749027252;
    goal.target_pose.pose.position.y = 38.5246810913;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.722836156881;
    goal.target_pose.pose.orientation.w = 0.691019457255;

    ROS_INFO("Sending Goal 11");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 11");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 11");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ทางเดินซ้าย
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.90473866463;
    goal.target_pose.pose.position.y = 45.5818634033;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.0141668618915;
    goal.target_pose.pose.orientation.w = 0.999899644977;

    ROS_INFO("Sending Goal 12");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 12");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 12");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ระเบียงซ้าย
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 18.0558700562;
    goal.target_pose.pose.position.y = 46.1546669006;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.200210509479;
    goal.target_pose.pose.orientation.w = 0.979752903489;

    ROS_INFO("Sending Goal 13");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 13");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 13");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ข้างห้องอาจารย์ 2 ทางยาว
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.885285925865;
    goal.target_pose.pose.position.y = 457871513367;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.72616865765;
    goal.target_pose.pose.orientation.w = 0.687516603907;

    ROS_INFO("Sending Goal 14");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 14");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 14");
    }

    ros::Duration(3).sleep();   

    goal.target_pose.header.frame_id = "map";  //โต๊ะหน้าห้องอาจารย์ 2 
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.534348666668;
    goal.target_pose.pose.position.y = 31.0463066101;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.70296576471;
    goal.target_pose.pose.orientation.w = 0.711223687489;

    ROS_INFO("Sending small 3");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 3");
    } else {
        ROS_INFO("The base failed to move to small 3");
    }

    goal.target_pose.header.frame_id = "map";  //โต๊ะหน้าห้องโปรเจ็กต์
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.871339201927;
    goal.target_pose.pose.position.y = 21.2429924011;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.713238523153;
    goal.target_pose.pose.orientation.w = 0.70092140008;

    ROS_INFO("Sending Goal 15");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 15");
        takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to goal 15");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //หน้า origin
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.25926506519;
    goal.target_pose.pose.position.y = 1.14333617687;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.691019445767;
    goal.target_pose.pose.orientation.w = 0.722836167864;

    ROS_INFO("Sending small 4");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 4");
    } else {
        ROS_INFO("The base failed to move to small 4");
    }

    goal.target_pose.header.frame_id = "map";  //โต๊ะหน้าห้องอาจารย์ 2 
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO("Sending Origin");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to Origin");
    } else {
        ROS_INFO("The base failed to move to Origin");
    }
    return 0;
}

