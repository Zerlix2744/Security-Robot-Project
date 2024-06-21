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
    cv::VideoCapture cap(1);
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

    goal.target_pose.pose.position.x = 1.565;
    goal.target_pose.pose.position.y = -12.545;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.005;
    goal.target_pose.pose.orientation.w = 0.999;

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

    goal.target_pose.pose.position.x = 17.428;
    goal.target_pose.pose.position.y = -12.799;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.003;
    goal.target_pose.pose.orientation.w = 0.999;

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

    goal.target_pose.pose.position.x = 17.428;
    goal.target_pose.pose.position.y = -12.799;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.718;
    goal.target_pose.pose.orientation.w = 0.696;

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

    goal.target_pose.pose.position.x = 17.428;
    goal.target_pose.pose.position.y = -12.799;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.999;
    goal.target_pose.pose.orientation.w = -0.004;

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

    goal.target_pose.pose.position.x = 9.212;
    goal.target_pose.pose.position.y = -12.732;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.711;
    goal.target_pose.pose.orientation.w = 0.704;

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

    goal.target_pose.pose.position.x = 1.690;
    goal.target_pose.pose.position.y = -12.657;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.999;
    goal.target_pose.pose.orientation.w = -0.009;

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

    goal.target_pose.pose.position.x = 1.690;
    goal.target_pose.pose.position.y = -12.657;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.716;
    goal.target_pose.pose.orientation.w = 0.698;

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

    goal.target_pose.pose.position.x = 2.389;
    goal.target_pose.pose.position.y = 0.794;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.938;
    goal.target_pose.pose.orientation.w = 0.347;

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

    goal.target_pose.pose.position.x = 1.135;
    goal.target_pose.pose.position.y = 10.154;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.725;
    goal.target_pose.pose.orientation.w = 0.688;

    ROS_INFO("Sending small 1");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 1");
    } else {
        ROS_INFO("The base failed to move to small 1");
    }
    ros::Duration(2).sleep();    

    goal.target_pose.header.frame_id = "map";  //จุดเลยโต๊ะ
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.135;
    goal.target_pose.pose.position.y = 14.605;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.640;
    goal.target_pose.pose.orientation.w = 0.768;

    ROS_INFO("Sending smaller 1");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to smaller 1");
    } else {
        ROS_INFO("The base failed to move to smaller 1");
    }
    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //ห้องเจ็กต์
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.354;
    goal.target_pose.pose.position.y = 17.518;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.958;
    goal.target_pose.pose.orientation.w = 0.286;

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

    goal.target_pose.header.frame_id = "map";  //จุดมากลาง
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.899;
    goal.target_pose.pose.position.y = 23.277;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.722;
    goal.target_pose.pose.orientation.w = 0.691;

    ROS_INFO("Sending smaller 2");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to smaller 2");
    } else {
        ROS_INFO("The base failed to move to smaller 2");
    }
    ros::Duration(2).sleep();    
    
    goal.target_pose.header.frame_id = "map";  //จุดก่อนโต๊ะ
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.687;
    goal.target_pose.pose.position.y = 26.663;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.738;
    goal.target_pose.pose.orientation.w = 0.675;

    ROS_INFO("Sending smaller 3");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to smaller 3");
    } else {
        ROS_INFO("The base failed to move to smaller 3");
    }
    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //จุดเล็กเลยเสา 2
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.819;
    goal.target_pose.pose.position.y = 32.559;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.723;
    goal.target_pose.pose.orientation.w = 0.691;

    ROS_INFO("Sending small 2");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 2");
	    takePhoto("/tmp/goal1.jpg");
        sendImageToLine("/tmp/goal1.jpg", "fhjpkrB9MUtHb15xhHj5Wy7TXXC9Dl5hUJHMhqi0KBX");
    } else {
        ROS_INFO("The base failed to move to small 2");
    }

    ros::Duration(3).sleep();

    goal.target_pose.header.frame_id = "map";  //ห้องอาจารย์ 2
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.138;
    goal.target_pose.pose.position.y = 39.406;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.722;
    goal.target_pose.pose.orientation.w = 0.691;

    ROS_INFO("Sending Goal 11");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to goal 11");
    } else {
        ROS_INFO("The base failed to move to goal 11");
    }

    ros::Duration(2).sleep();

    


    goal.target_pose.header.frame_id = "map";  //ทางเดินซ้าย
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 3.006;
    goal.target_pose.pose.position.y = 45.382;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

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

    goal.target_pose.header.frame_id = "map";  //จุดซอยระเบียง 1 
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 9.473;
    goal.target_pose.pose.position.y = 45.496;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending smaller 4");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to smaller 4");
    } else {
        ROS_INFO("The base failed to move to smaller 4");
    }

    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //จุดซอยระเบียง 2 
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 14.806;
    goal.target_pose.pose.position.y = 45.382;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending smaller 5");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to smaller 5");
    } else {
        ROS_INFO("The base failed to move to smaller 5");
    }

    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //ระเบียงซ้าย
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 18.611;
    goal.target_pose.pose.position.y = 45.444;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.218;
    goal.target_pose.pose.orientation.w = 0.976;

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

    goal.target_pose.header.frame_id = "map";  //กลับ จุดซอยระเบียง 1
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 12.310;
    goal.target_pose.pose.position.y = 45.382;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;

    ROS_INFO("Sending smaller 6");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to smaller 6");
    } else {
        ROS_INFO("The base failed to move to smaller 6");
    }

    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //กลับ จุดซอยระเบียง 2
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 5.956;
    goal.target_pose.pose.position.y = 45.042;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;

    ROS_INFO("Sending smaller 6");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to smaller 6");
    } else {
        ROS_INFO("The base failed to move to smaller 6");
    }

    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //ข้างห้องอาจารย์ 2 ทางยาว
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.779;
    goal.target_pose.pose.position.y = 43.113;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.726;
    goal.target_pose.pose.orientation.w = 0.688;

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

    goal.target_pose.pose.position.x = 0.534;
    goal.target_pose.pose.position.y = 31.046;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.703;
    goal.target_pose.pose.orientation.w = 0.711;

    ROS_INFO("Sending small 3");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 3");
    } else {
        ROS_INFO("The base failed to move to small 3");
    }

    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //โต๊ะหน้าห้องโปรเจ็กต์
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.871;
    goal.target_pose.pose.position.y = 21.242;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.713;
    goal.target_pose.pose.orientation.w = 0.701;

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

    goal.target_pose.pose.position.x = 1.259;
    goal.target_pose.pose.position.y = 1.143;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.691;
    goal.target_pose.pose.orientation.w = 0.722;

    ROS_INFO("Sending small 4");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to small 4");
    } else {
        ROS_INFO("The base failed to move to small 4");
    }

    ros::Duration(2).sleep();

    goal.target_pose.header.frame_id = "map";  //origin
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

