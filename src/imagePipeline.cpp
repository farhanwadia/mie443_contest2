#include <imagePipeline.h>
#include <string>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}
double ImagePipeline::matchToTemplate(Mat img_object){
    //convert image to grayscale
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    
    //--Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint>keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
    //detector->detectAndCompute(img, Mat(), keypoints_scene, descriptors_scene);
    detector->detectAndCompute(gray_img, Mat(), keypoints_scene, descriptors_scene);

    //Lowe's ratio filer
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
            good_matches.push_back(knn_matches[i][0]);
        }
    }
        
    Mat img_matches;

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    Mat H = findHomography( obj, scene, RANSAC );

    //-- Get the corners from the img_object ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
    obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<Point2f> scene_corners(4);

    // Define scene_corners using Homography
    try {
        perspectiveTransform(obj_corners, scene_corners, H);
    } catch (Exception& e) {
        ;
    }

    // Define a contour using the scene_corners
    std::vector<Point2f> contour;
    for (int i = 0; i < 4; i++){
	    contour.push_back(scene_corners[i] + Point2f( img_object.cols, 0));
    }
    double area = contourArea(contour);
    double area_weight = 1.0;
    if (area >  1000*1000 || area < 5 * 5)
    {
        area_weight = 0.0;
    }
    else
    {
        area_weight = 1.0;
    }
    std::cout << "area: " << area << ", weight: " << area_weight << std::endl;

    double indicator;
    std::vector< DMatch > best_matches;

    // Check if the good match is inside the contour.
    Point2f matched_point;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        matched_point = keypoints_scene[ good_matches[i].trainIdx ].pt + Point2f( img_object.cols, 0);
        indicator = pointPolygonTest(contour, matched_point, false);
        if(indicator >= 0) best_matches.push_back( good_matches[i]);
    }
    
    drawMatches( img_object, keypoints_object, gray_img, keypoints_scene,
                 best_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    
    //-- Show detected matches - DON'T NEED NOW
    //imshow( "Good Matches & Object detection", img_matches );
    cv::waitKey(10);

    return (double)best_matches.size()*area_weight;
    //return (double)good_matches.size();
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    double best_matches, matches;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } 
    else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } 
    else {
        // Records the best match, also if all matches less than this value, then probably blank
        best_matches = 25;
        template_id = -1;

        // For each box template
        for (int i = 0; i < boxes.templates.size(); ++i)
        {
            // Get the number of matches between the view and current box template
            matches = matchToTemplate(boxes.templates[i]);
            std::cout  << matches << " matching features for template " << i << std::endl;

            // Save the maximum matches and corresponding box index
            if (matches > best_matches){
                best_matches = matches;
                template_id = i;
            } 
        }
    }
    // Get grayscale image
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    
    // For displaying the image
    cv::imshow("view", gray_img);
    cv::waitKey(1000);
    std::cout  << "Best template is  " << template_id << " with " << best_matches << " matches" << std::endl;

    return template_id;
}