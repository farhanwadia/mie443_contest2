#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <iostream>
#include <fstream>

#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)


float dist(float x1, float y1, float x2, float y2){
    //Calculates the Euclidean distance between two points (x1, y1), (x2, y2)
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

float add2Pi(float angle){
    //Converts angles from -pi to 0 to be between pi and 2pi
    if (angle < 0){
        angle = angle + 2*M_PI;
    }
    return angle;
}

float minus2Pi(float angle){
    //Converts angles from pi to 2pi to be between -pi and 0 
    if(angle > M_PI){
        angle = angle - 2*M_PI;
    }
    return angle;
}

void fillNavCoords(float nav_coords[10][3], Boxes* pBoxes, float offset){
    //This function fills in the nav_coords array using boxes.coords
    //New x,y,z are calculated such that the turtlebot faces in front of the box at offset distance
    Boxes boxes;
    boxes = *pBoxes;
    for(int i=0; i < boxes.coords.size(); i++){
        nav_coords[i][0] = boxes.coords[i][0] + offset*cosf(boxes.coords[i][2]); //set x value
        nav_coords[i][1] = boxes.coords[i][1] + offset*sinf(boxes.coords[i][2]); //set y value
        nav_coords[i][2] = minus2Pi(boxes.coords[i][2] + M_PI); //set angle
    }
}

void fillAdjacencyMatrix(float adjMat[10][10], float nav_coords[10][3]){
    //Fills adjacency matrix of graph edge weights (i.e. distance between nodes i, j)
    for(int i=0; i<10; i++){
        for(int j=0; j<10; j++){
            adjMat[i][j] = dist(nav_coords[i][0], nav_coords[i][1], nav_coords[j][0], nav_coords[j][1]);
            ROS_INFO("Distance (%d, %d): %.3f", i, j, adjMat[i][j]);
        }
    }
}

int findClosestBoxAtStart(float nav_coords[10][3]){
    float minD = std::numeric_limits<float>::infinity();
    float d;
    int argmin;
    for(int i=0; i<10; i++){
        d = dist(0, 0, nav_coords[i][0], nav_coords[i][1]);
        ROS_INFO("Distance to node %d: %.3f", i, d);
        if (d < minD){
            minD = d;
            argmin = i;
        }
    }
    return argmin;
}

float bruteForceTSP(float nav_coords[10][3], float adjMat[10][10], int source, std::vector<int> &TSPTour){
    // Returns the distance of the optimal TSP tour and modifies vector TSPTour to provide the nodes in the order of the tour path
    // Adapted from https://iq.opengenus.org/travelling-salesman-problem-brute-force/
    std::vector<int> nodes;
    int num_nodes = 10;

    // Append the other nodes to the vector
    for(int i=0; i<num_nodes; i++){
        if(i != source){
            nodes.push_back(i);
        }
    }
    int n = nodes.size();
    float shortestPathWgt = std::numeric_limits<float>::infinity();

    // Generate permutations and track the minimum
    while(next_permutation(nodes.begin(),nodes.end())){
        float currentPathWgt = 0;
        std::vector<int> currentTour;

        int j = source;
        currentTour.push_back(source);
        
        // Calculate distance and visiting order for current tour path
        for (int i = 0; i < n; i++)
        {
            currentPathWgt += adjMat[j][nodes[i]];
            j = nodes[i];
            currentTour.push_back(j);
        }
        currentPathWgt += adjMat[j][source]; //add the distance from last node back to start

        // Update shortest path and the node order if our current tour is smaller than the previous minimum
        if (currentPathWgt < shortestPathWgt){
            shortestPathWgt = currentPathWgt;
            TSPTour = currentTour;
        }
    }
    return shortestPathWgt;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    
    float adjMat[10][10];
    float nav_coords[10][3];
    int startBox, currentNode = 0;
    float xx, yy, zz, dz, offset = 0.35, timeout = 10, TSPDist;
    bool nav_success;
    std::vector<int> TSPTour;

    // Initialize output file to write image IDs to
    std::ofstream output("Group18_BoxIDs.txt");

    //Contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    //Fill the nav_coords array
    fillNavCoords(nav_coords, &boxes, offset);
    for(int i =0; i < 10; i++){
        ROS_INFO("Box %d: (%.3f, %.3f, %.3f)", i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
        ROS_INFO("Nav %d: (%.3f, %.3f, %.3f) \n", i, nav_coords[i][0], nav_coords[i][1], nav_coords[i][2]);
    }

    //Fill adjacency matrix of graph edge weights (i.e. distance between nodes i, j)
    fillAdjacencyMatrix(adjMat, nav_coords);

    //Find closest box to current location. Start the tour from there
    startBox = findClosestBoxAtStart(nav_coords);
    ROS_INFO("Start Box: %d", startBox);

    //Brute Force TSP. TSP
    TSPDist = bruteForceTSP(nav_coords, adjMat, startBox, TSPTour);
    ROS_INFO("TSP Distance: %.5f \n Printing Nodes:", TSPDist);
    for(int i=0; i<TSPTour.size(); i++){
        ROS_INFO("%d", TSPTour[i]);
    }
    
    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.getTemplateID(boxes);
        
        //Travel to the nodes and then back to the start
        if (currentNode <= 10){ 
            if (currentNode < 10){
                xx = nav_coords[TSPTour[currentNode]][0];
                yy = nav_coords[TSPTour[currentNode]][1];
                zz = nav_coords[TSPTour[currentNode]][2];
            }
            else {
                //Return to start after exploring all nodes
                xx = 0;
                yy = 0;
                zz = 0;
            }
            ROS_INFO("Navigating to node %d. (%.3f, %.3f, %.3f)", currentNode, xx, yy, zz);
            nav_success = Navigation::moveToGoal(xx, yy, zz, timeout);

            //Try varying the angle to be 30, -30, 60, -60 from centre if navigation was unsuccesful
            dz = DEG2RAD(30);     
            while(!nav_success && fabs(dz) <=DEG2RAD(61) && currentNode < 10){
                // Recalculate xx, yy, zz, to incorporate angle offst dz
                xx = boxes.coords[TSPTour[currentNode]][0] + offset*cosf(boxes.coords[TSPTour[currentNode]][2] + dz);
                yy = boxes.coords[TSPTour[currentNode]][1] + offset*sinf(boxes.coords[TSPTour[currentNode]][2] + dz);
                zz = minus2Pi(boxes.coords[TSPTour[currentNode]][2] + dz + M_PI);
                //Try navigating
                ROS_INFO("Navigating to node %d with offset %.1f. (%.3f, %.3f, %.3f)", currentNode, RAD2DEG(dz), xx, yy, zz);
                nav_success = Navigation::moveToGoal(xx, yy, zz, timeout);
                if(nav_success){
                    break;
                }
                //Change dz if navigation still unsuccesful
                if(dz > 0){
                    dz = -dz;
                }
                else{
                    dz = -dz;
                    dz = dz + DEG2RAD(30);
                }
            }
            if (fabs(dz) > DEG2RAD(61)){
                ROS_INFO("COULD NOT GET TO NODE %d. Nav Status: %d", currentNode, nav_success);
            }
            
            ROS_INFO("Finshed moving. Nav Status: %d", nav_success);
            currentNode ++;
        }
        

        // Write index and tag to output file. Note: ind and tag current placeholders
        // output << "Box: " << ind << " Tag: " << tag << std::endl;
    	// output << "Located at: (" << boxes.coords[ind-1][0] << ", " << boxes.coords[ind-1][1] << ", " << boxes.coords[ind-1][2] <<  ")" << std::endl;
    		
        




        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
