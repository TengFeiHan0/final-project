# AFOA


# Related article:
https://ieeexplore.ieee.org/document/7886853/  (If you use this project for your academic research, I reconmend you to  cite this paper.)

# Video:
https://www.youtube.com/watch?v=vIR7NgsMt6U , https://www.youtube.com/watch?v=Cx94iW5SrGY

# 1. Prerequisites
The codes works for AR.drone 2.0 or 1.0, we assume that you already have  a new one. However, if you couldn't buy it temporarily,  it's available that using a simulator(http://wiki.ros.org/tum_simulator) published by TUM Robotic Group to simulate the whole process.

OpenCV:https://opencv.org/  (reconmended version: less than 3.0.0)

Egien:http://eigen.tuxfamily.org/index.php?title=Main_Page

g2o: https://github.com/RainerKuemmerle/g2o

PCL:https://github.com/PointCloudLibrary/pcl  (1.8)

ardrone_autonomy(edited verison:https://github.com/tum-vision/ardrone_autonomy or original version:http://wiki.ros.org/ardrone_autonomy)

you also have to install other dependencies used for each libraries. In addition, you should create a new folder named Thirdparty under SLAM and put g2o and DBoW2 into it. Due to the limitation of the amount of uploading files, I have to upload g2o and DBow2 separately.


# 2. Testing environment
ros-indigo + Ubuntu 14.04(best choice)

ros-kinetic+Ubuntu 16.04(probably good)

please build up a rosbuild(http://wiki.ros.org/rosbuild) workspace to store these packages, if you want to use catkin, please edit and create relevant files. 

# 3. Compiling problems
It is not uncommon that you fail to compile one of packages and build up new executable files. If you really find some bugs , you could send an email to editers and original author directly.

# 4. Acknowledgements
We use g2o for non-linear optimization, DBoW2 for loop detection, and ORBSLAM2 for feature extraction.

# 5. Lienses
The source code is released under GPLv3 license.

We are still working on improving the code reliability. For any technical issues, please contact Omid <esrafilian.omid@gmail.com>, Mouaad <boughellaba.mouaad@gmail.com>, Tengfei Han<hantengfei007@gmail.com>.
 
      

      
