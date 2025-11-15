## TO ADD WORLD (NOT CUSTOM)

  1. <!-- World File  ==> دور علي مكان الباكج ==> كل باكج فيها مجموعة ورلدز -->
    <arg name="world_file" default="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"/>
  2. no need to mkdir world

## Find my robot Description

  **<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find diff_drive_description)/urdf/main_diff.xacro'"/>**
  * ده بيعمل تحميل (Load) لملف الـ URDF بتاع روبوتك في الـ ROS parameter server.
  * وده معناه إنك بتحط وصف الروبوت (links, joints, sensors, plugins...) في متغير اسمه robot_description.
  * now robot is recognized inside ros server

## Spwan born in Gazebo 

  * ده بيشغل Node اسمها spawn_model
  * تاخد الوصف اللي اتحمل في robot_description
    وتستخدمه لإنشاء روبوت فعلي داخل Gazebo.
  * الروبوت بتاعك يتولد (spawn) فعليًا داخل Gazebo world،



  