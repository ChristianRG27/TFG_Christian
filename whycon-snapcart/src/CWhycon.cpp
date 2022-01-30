#include "CWhycon.h"
#define PI 3.14159265

/*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
void CWhycon::manualcalibration(){
    if (currentSegmentArray[0].valid){
        STrackedObject o = objectArray[0];
        moveOne = moveVal;

        //object found - add to a buffer
        if (calibStep < calibrationSteps) calibTmp[calibStep++] = o;

        //does the buffer contain enough data to calculate the object position
        if (calibStep == calibrationSteps){
            o.x = o.y = o.z = 0;
            for (int k = 0;k<calibrationSteps;k++){
                o.x += calibTmp[k].x;
                o.y += calibTmp[k].y;
                o.z += calibTmp[k].z;
            }
            o.x = o.x/calibrationSteps;	
            o.y = o.y/calibrationSteps;	
            o.z = o.z/calibrationSteps;
            if (calibNum < 4){
                calib[calibNum++] = o;
            }

            //was it the last object needed to establish the transform ?
            if (calibNum == 4){
                //calculate and save transforms
                trans->calibrate2D(calib,fieldLength,fieldWidth);
                trans->calibrate3D(calib,fieldLength,fieldWidth);
                calibNum++;
                numMarkers = wasMarkers;
                trans->saveCalibration(calibDefPath.c_str());
                trans->transformType = lastTransformType;
                detectorArray[0]->localSearch = false;
            }
            calibStep++;
        }
    }
}

/*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
void CWhycon::autocalibration(){
    bool saveVals = true;
    for (int i = 0;i<numMarkers;i++){
        if (detectorArray[i]->lastTrackOK == false) saveVals=false;
    }
    if (saveVals){
        int index[] = {0,0,0,0};	
        int maxEval = 0;
        int eval = 0;
        int sX[] = {-1,+1,-1,+1};
        int sY[] = {+1,+1,-1,-1};
        for (int b = 0;b<4;b++){
            maxEval = -10000000;
            for (int i = 0;i<numMarkers;i++){
                eval = 	sX[b]*currentSegmentArray[i].x +sY[b]*currentSegmentArray[i].y;
                if (eval > maxEval){
                    maxEval = eval;
                    index[b] = i;
		    indexGUI[b] = i;
                }
            }
        }
        printf("INDEX: %i %i %i %i\n",index[0],index[1],index[2],index[3]);
        for (int i = 0;i<4;i++){
            if (calibStep <= autoCalibrationPreSteps) calib[i].x = calib[i].y = calib[i].z = 0;
            calib[i].x+=objectArray[index[i]].x;
            calib[i].y+=objectArray[index[i]].y;
            calib[i].z+=objectArray[index[i]].z;
        }
        calibStep++;
        if (calibStep == autoCalibrationSteps){
            for (int i = 0;i<4;i++){
                calib[i].x = calib[i].x/(autoCalibrationSteps-autoCalibrationPreSteps);
                calib[i].y = calib[i].y/(autoCalibrationSteps-autoCalibrationPreSteps);
                calib[i].z = calib[i].z/(autoCalibrationSteps-autoCalibrationPreSteps);
            }
            trans->calibrate2D(calib,fieldLength,fieldWidth);
            trans->calibrate3D(calib,fieldLength,fieldWidth);
            calibNum++;
            numMarkers = wasMarkers;
            trans->saveCalibration(calibDefPath.c_str());
            trans->transformType = lastTransformType;
            autocalibrate = false;
        }
        
    }
}

/*process events coming from GUI*/
void CWhycon::processKeys(){
    //process mouse - mainly for manual calibration - by clicking four circles at the corners of the operational area 
    while (SDL_PollEvent(&event)){
        if (event.type == SDL_MOUSEBUTTONDOWN){
            if (calibNum < 4 && calibStep > calibrationSteps){
                calibStep = 0;
                trans->transformType = TRANSFORM_NONE;
            }
            if (numMarkers > 0){
                currentSegmentArray[numMarkers-1].x = event.motion.x*guiScale; 
                currentSegmentArray[numMarkers-1].y = event.motion.y*guiScale;
                currentSegmentArray[numMarkers-1].valid = true;
                detectorArray[numMarkers-1]->localSearch = true;
            }
        }
    }

    //process keys 
    keys = SDL_GetKeyState(&keyNumber);
    bool shiftPressed = keys[SDLK_RSHIFT] || keys[SDLK_LSHIFT];
    bool pPressed = keys[SDLK_p];

    //program control - (s)top, (p)ause+move one frame and resume

    // Se procesan las diferentes teclas con diferentes acciones. 
    if (keys[SDLK_ESCAPE]) stop = true;
    if (keys[SDLK_SPACE] && lastKeys[SDLK_SPACE] == false){ moveOne = 100000000; moveVal = 10000000;};
    if (keys[SDLK_p] && lastKeys[SDLK_p] == false) {moveOne = 1; moveVal = 0;}

    if (keys[SDLK_m] && lastKeys[SDLK_m] == false) printf("SAVE %02f %02f %02f %02f %02f %02f %02f\n",objectArray[0].x,objectArray[0].y,objectArray[0].z,objectArray[0].d,currentSegmentArray[0].m0/currentSegmentArray[0].m1, currentSegmentArray[0].m0,currentSegmentArray[0].m1);
    if (keys[SDLK_n] && lastKeys[SDLK_n] == false) printf("SEGM %02f %02f %02f %02f %02f\n",currentSegmentArray[0].x,currentSegmentArray[0].y,currentSegmentArray[0].m0/currentSegmentArray[0].m1,currentSegmentArray[0].m0,currentSegmentArray[0].m1);
    if (keys[SDLK_s] && lastKeys[SDLK_s] == false) image->saveBmp();

    if ((keys[SDLK_2] || keys[SDLK_4] ||keys[SDLK_6] ||keys[SDLK_8]) && pPressed) {

            // pointer demo to FILE
	    FILE* demo;

	    int idSelected = 1;

	    if (keys[SDLK_2])      { idSelected = 2; demo = fopen("/home/viki/catkin_ws/src/whycon-snapcart/documentos/rutaRobot_2.txt", "w+"); }
	    else if (keys[SDLK_4]) { idSelected = 4; demo = fopen("/home/viki/catkin_ws/src/whycon-snapcart/documentos/rutaRobot_4.txt", "w+"); }
	    else if (keys[SDLK_6]) { idSelected = 6; demo = fopen("/home/viki/catkin_ws/src/whycon-snapcart/documentos/rutaRobot_6.txt", "w+"); }
	    else if (keys[SDLK_8]) { idSelected = 8; demo = fopen("/home/viki/catkin_ws/src/whycon-snapcart/documentos/rutaRobot_8.txt", "w+"); }

	    std::vector<std::vector<float>> dataPath = gui->getDataPath(idSelected);

	    int i = 0;
  	    while (i<dataPath.size()){
	
		currentSegmentArrayPath[i].x     = dataPath[i][0];
		currentSegmentArrayPath[i].y     = dataPath[i][1];
		currentSegmentArrayPath[i].v0    = dataPath[i][2];
		currentSegmentArrayPath[i].v1    = dataPath[i][3];
		currentSegmentArrayPath[i].m0    = dataPath[i][4];
		currentSegmentArrayPath[i].m1    = dataPath[i][5];
		currentSegmentArrayPath[i].angle = dataPath[i][6];
		currentSegmentArrayPath[i].ID    = 2;

		objectArrayPath[i] = trans->transform(currentSegmentArrayPath[i]);
		fprintf(demo, "%f %f\n", objectArrayPath[i].x, objectArrayPath[i].y);
 		i++;

	    }

	    fclose(demo);	// closes the file pointed by demo
	 
    }

    //initiate autocalibration (searches for 4 outermost circular patterns and uses them to establisht the coordinate system)
    if (keys[SDLK_a] && lastKeys[SDLK_a] == false) { calibStep = 0; lastTransformType=trans->transformType; wasMarkers = numMarkers; autocalibrate = true;trans->transformType=TRANSFORM_NONE;}; 

    //manual calibration (click the 4 calibration circles with mouse)
    if (keys[SDLK_r] && lastKeys[SDLK_r] == false) { calibNum = 0; wasMarkers=numMarkers; numMarkers = 1;}

    //debugging - toggle drawing coordinates and debugging results results
    if (keys[SDLK_l] && lastKeys[SDLK_l] == false) drawCoords = drawCoords == false;
    if (keys[SDLK_d] && lastKeys[SDLK_d] == false){ 
        for (int i = 0;i<numMarkers;i++){
            detectorArray[i]->draw = detectorArray[i]->draw==false;
            detectorArray[i]->debug = detectorArray[i]->debug==false;
            decoder->debugSegment = decoder->debugSegment==false;
        }
    }

    //transformations to use - in our case, the relevant transform is '2D'
    if (pPressed == false) {if (keys[SDLK_1] && lastKeys[SDLK_1] == false) trans->transformType = TRANSFORM_NONE;}
    if (pPressed == false) {if (keys[SDLK_2] && lastKeys[SDLK_2] == false) trans->transformType = TRANSFORM_2D;}
    if (pPressed == false) {if (keys[SDLK_3] && lastKeys[SDLK_3] == false) trans->transformType = TRANSFORM_3D;}

    // TODO camera low-level settings 

    //display help
    if (keys[SDLK_h] && lastKeys[SDLK_h] == false) displayHelp = displayHelp == false; 

    //adjust the number of robots to be searched for
    if (keys[SDLK_PLUS]) numMarkers++;
    if (keys[SDLK_EQUALS]) numMarkers++;
    if (keys[SDLK_MINUS]) numMarkers--;
    if (keys[SDLK_KP_PLUS]) numMarkers++;
    if (keys[SDLK_KP_MINUS]) numMarkers--;
    if (keys[SDLK_KP_MINUS]) numMarkers--;

    if (keys[SDLK_t] && lastKeys[SDLK_t] == true) drawPath = drawPath == false;
    if (keys[SDLK_t] && lastKeys[SDLK_t] == false) drawPath = drawPath == true;

	if (drawPath) {ROS_INFO("Activado drawpath");}



    //store the key states
    memcpy(lastKeys,keys,keyNumber);
}

void CWhycon::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
    if(msg->K[0] == 0)
    {
        ROS_ERROR_ONCE("ERROR: Camera is not calibrated!");
        return;
    }
    else if(msg->K[0] != intrinsic.at<float>(0,0) || msg->K[2] != intrinsic.at<float>(0,2) || msg->K[4] != intrinsic.at<float>(1,1) ||  msg->K[5] != intrinsic.at<float>(1,2))
    {
        for(int i = 0; i < 5; i++) distCoeffs.at<float>(i) = msg->D[i];
        int tmpIdx = 0;
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                intrinsic.at<float>(i, j) = msg->K[tmpIdx++];
            }
        }
        trans->updateParams(intrinsic, distCoeffs);
    }
}

void CWhycon::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    //setup timers to assess system performance
    CTimer timer;
    timer.reset();
    timer.start();

    CTimer globalTimer;
    globalTimer.reset();
    globalTimer.start();

    // check if readjusting of camera is needed
    if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
        delete image;
        ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",
                image->width, image->height, image->bpp, msg->width, msg->height, msg->step/msg->width);
        image = new CRawImage(msg->width,msg->height,msg->step/msg->width);
        if(useGui){
            while(image->height/guiScale > screenHeight || image->height/guiScale > screenWidth) guiScale = guiScale*2;
            if(gui == NULL){
                gui = new CGui(msg->width, msg->height, guiScale, fontPath.c_str());
            }else{
                delete gui;
                gui = new CGui(msg->width, msg->height, guiScale, fontPath.c_str());
            }
        }
    }

    memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);

    numFound = numStatic = 0;
    timer.reset();

    // track the robots found in the last attempt 
    for (int i = 0;i<numMarkers;i++){
        if (currentSegmentArray[i].valid){
            lastSegmentArray[i] = currentSegmentArray[i];
            currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
            currInnerSegArray[i] = detectorArray[i]->getInnerSegment();
        }
    }

    // search for untracked (not detected in the last frame) robots 
    for (int i = 0;i<numMarkers;i++){
        if (currentSegmentArray[i].valid == false){
            lastSegmentArray[i].valid = false;
            currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
            currInnerSegArray[i] = detectorArray[i]->getInnerSegment();
        }
        if (currentSegmentArray[i].valid == false) break;		//does not make sense to search for more patterns if the last one was not found
    }
	
    float dmin=1000;
    // perform transformations from camera to world coordinates
    for (int i = 0;i<numMarkers;i++){
        if (currentSegmentArray[i].valid){
            int step = image->bpp;
            int pos;
	    double x, y, result;
		//c ROS_INFO("currentSegmentArray[%d].x: %f, currentSegmentArray[%d].y: %f",i, currentSegmentArray[i].x, i, currentSegmentArray[i].y);
		
  		//c printf ("The arc tangent for (x=%f, y=%f) is %f degrees\n", x, y, result );
		
		//c gui->drawLine(currentSegmentArray[0].x,currentSegmentArray[1].x,currentSegmentArray[0].y,currentSegmentArray[1].y);
		//c cv::line(image, cv::Point(currentSegmentArray[0].x,currentSegmentArray[0].y), cv::Point(currentSegmentArray[1].x,currentSegmentArray[1].y), cv::Scalar(0,255,255), 2, 8);
            pos = ((int)currentSegmentArray[i].x+((int)currentSegmentArray[i].y)*image->width);
	
            image->data[step*pos+0] = 255;
            image->data[step*pos+1] = 0;
            image->data[step*pos+2] = 0;
            pos = ((int)currInnerSegArray[i].x+((int)currInnerSegArray[i].y)*image->width);
            image->data[step*pos+0] = 0;
            image->data[step*pos+1] = 255;
            image->data[step*pos+2] = 0;

            objectArray[i] = trans->transform(currentSegmentArray[i]);
	   
            if(identify){
                //c int segmentID = decoder->identifySegment(&currentSegmentArray[i], &objectArray[i], image) + 1;
                //c //                if (debug) printf("SEGMENT ID: %i\n", segmentID);
                //c if (segmentID > -1){
                //c     //objectArray[i].yaw = currentSegmentArray[i].angle;
                //c     objectArray[i].ID = currentSegmentArray[i].ID = segmentID;
                //c }else{
                //c     //currentSegmentArray[i].angle = lastSegmentArray[i].angle;
                //c     objectArray[i].ID = currentSegmentArray[i].ID = lastSegmentArray[i].ID;
                //c }
		
		float segmentID = currentSegmentArray[i].m0/currentSegmentArray[i].m1;
		
		
		float d0 = 12/currentSegmentArray[i].m0;
		float d1 =  3/currentSegmentArray[i].m1;

		float currentSegmentArrayM0corregido = currentSegmentArray[i].m0*d0;
		float currentSegmentArrayM1corregido=  currentSegmentArray[i].m1*d1;
		float segmentIDCorregido = currentSegmentArrayM0corregido/currentSegmentArrayM1corregido;
		
		//ROS_INFO("currentSegmentArray[%i].m0: %f ", i, currentSegmentArray[i].m0);
		//ROS_INFO("currentSegmentArray[%i].m1: %f \n", i, currentSegmentArray[i].m1);
		//ROS_INFO("segmentID: %f \n",segmentID);
		//float segmentIDCorregido = currentSegmentArrayXcorregido/currentSegmentArrayYcorregido;
		//ROS_INFO("objectArray[i].x: %f",objectArray[i].x);
		//ROS_INFO("objectArray[i].y: %f",objectArray[i].y);
		//ROS_INFO("currentSegmentArray[i].m0 corregido: %f", currentSegmentArrayM0corregido);
		//ROS_INFO("currentSegmentArray[i].m1 corregido: %f",currentSegmentArrayM1corregido);
		//ROS_INFO("segmentID corregido: %f \n",currentSegmentArrayXcorregido/currentSegmentArrayYcorregido);
		

		if (segmentID >= 4.6){
			objectArray[i].ID = currentSegmentArray[i].ID = 2;
		} else if (segmentID < 4.6 && segmentID >= 3.375) {
			objectArray[i].ID = currentSegmentArray[i].ID = 4;
		} else if (segmentID < 3.375 && segmentID >= 2.4) {
			objectArray[i].ID = currentSegmentArray[i].ID = 6;
		} else if (segmentID < 2.4 && segmentID >=1.5) {
			objectArray[i].ID = currentSegmentArray[i].ID = 8;
		} else {
			objectArray[i].ID = currentSegmentArray[i].ID = 0;
		}


		if (objectArray[i].ID != 0){
			for (int j = 0;j<numMarkers;j++) {
					//ROS_INFO("Distancia entre centro de elipse y centro de baliza: %f ", sqrt((currentSegmentArray[j].x-currentSegmentArray[i].x)*(currentSegmentArray[j].x-currentSegmentArray[i].x) + (currentSegmentArray[j].y-currentSegmentArray[i].y)*(currentSegmentArray[j].y-currentSegmentArray[i].y)));
				if (objectArray[j].ID == 0 && sqrt((currentSegmentArray[j].x-currentSegmentArray[i].x)*(currentSegmentArray[j].x-currentSegmentArray[i].x) + (currentSegmentArray[j].y-currentSegmentArray[i].y)*(currentSegmentArray[j].y-currentSegmentArray[i].y)) < 70){

					
					result = -(atan2(-currentSegmentArray[j].y+currentSegmentArray[i].y, currentSegmentArray[j].x-currentSegmentArray[i].x)*180/PI - 180) - angleWorkZone;
					if (result < 0) {
						result = result + 360; 
					} else if (result > 360) {
						result = result - 360;
					}

					objectArray[i].yaw = result;

					/*ROS_INFO("Distancia entre centro de elipse y centro de baliza: %f ", sqrt((currentSegmentArray[j].x-currentSegmentArray[i].x)*(currentSegmentArray[j].x-currentSegmentArray[i].x) + (currentSegmentArray[j].y-currentSegmentArray[i].y)*(currentSegmentArray[j].y-currentSegmentArray[i].y)));
					ROS_INFO("Distancia entre puntos transformados: tX: %f, tY: %f ", objectArray[j].x-objectArray[i].x, objectArray[j].y-objectArray[i].y);
					ROS_INFO("currentSegmentArray[j].x: %f, currentSegmentArray[j].y: %f ", currentSegmentArray[j].x, currentSegmentArray[j].y);
					ROS_INFO("currentSegmentArray[i].x: %f, currentSegmentArray[i].y: %f ", currentSegmentArray[i].x, currentSegmentArray[i].y);
					ROS_INFO("currentSegmentArray.xDelta: %f, currentSegmentArray.yDelta: %f para i= %d y j= %d", currentSegmentArray[j].x-currentSegmentArray[i].x, currentSegmentArray[j].y-currentSegmentArray[i].y, i, j);
					ROS_INFO("Result: %f \n", result);*/
				}
			}
		}

		
		
	    	

		//c gui->drawLine(objectArray[0].x,objectArray[1].x,objectArray[0].y,objectArray[1].y);

            }else{
                float dist1 = sqrt((currInnerSegArray[i].x-objectArray[i].segX1)*(currInnerSegArray[i].x-objectArray[i].segX1)+(currInnerSegArray[i].y-objectArray[i].segY1)*(currInnerSegArray[i].y-objectArray[i].segY1));
                float dist2 = sqrt((currInnerSegArray[i].x-objectArray[i].segX2)*(currInnerSegArray[i].x-objectArray[i].segX2)+(currInnerSegArray[i].y-objectArray[i].segY2)*(currInnerSegArray[i].y-objectArray[i].segY2));
                if(dist1 < dist2){
                    currentSegmentArray[i].x = objectArray[i].segX1;
                    currentSegmentArray[i].y = objectArray[i].segY1;
                    objectArray[i].x = objectArray[i].x1;
                    objectArray[i].y = objectArray[i].y1;
                    objectArray[i].z = objectArray[i].z1;
                    //c objectArray[i].pitch = objectArray[i].pitch1;
                    //c objectArray[i].roll = objectArray[i].roll1;
                    //c objectArray[i].yaw = objectArray[i].yaw1;
                }else{
                    currentSegmentArray[i].x = objectArray[i].segX2;
                    currentSegmentArray[i].y = objectArray[i].segY2;
                    objectArray[i].x = objectArray[i].x2;
                    objectArray[i].y = objectArray[i].y2;
                    objectArray[i].z = objectArray[i].z2;
                    //c objectArray[i].pitch = objectArray[i].pitch2;
                    //c objectArray[i].roll = objectArray[i].roll2;
                    //c objectArray[i].yaw = objectArray[i].yaw2;
                }
            }

            numFound++;
            if (currentSegmentArray[i].x == lastSegmentArray[i].x) numStatic++;

        }
    }
    //    if(numFound > 0) ROS_INFO("Pattern detection time: %i us. Found: %i Static: %i.",globalTimer.getTime(),numFound,numStatic);
    evalTime = timer.getTime();

    // publishing information about tags 
    whycon_ros::MarkerArray markerArray;
    
    // Whycon_marker should have the same header as the image received
    markerArray.header = msg->header;
    
    visualization_msgs::MarkerArray visualArray;

    for (int i = 0; i < numMarkers; i++){
        if (currentSegmentArray[i].valid){
            // printf("ID %d\n", currentSegmentArray[i].ID);
            whycon_ros::Marker marker;

            marker.id = currentSegmentArray[i].ID;
            marker.size = currentSegmentArray[i].size;

            // Convert to ROS standard Coordinate System
            marker.position.position.x = -objectArray[i].y;
            marker.position.position.y = -objectArray[i].z;
            marker.position.position.z = objectArray[i].x;

            //double data[4] = {marker.id*100,marker.position.position.x,marker.position.position.y,marker.position.position.z}; //TODO
            double data[4] = {marker.id*10000,objectArray[i].x,objectArray[i].y,objectArray[i].z}; //TODO
            Mat descriptor = cv::Mat(cv::Size(1,4), CV_64FC1,data);
            

            // the "roll, pitch, yaw" vector (in rotation), basically this is the normal of the marker in image coords
            // as currently reported in "rotation" field
            tf::Vector3 axis_vector(0.0, 0.0, objectArray[i].yaw);

            //the "up vector" (pointing out of the camera, this is the reference the orientation is based on)
            tf::Vector3 up_vector(0.0, 0.0, 1.0);

            // the position of the marker (normalised to unit length)
            tf::Vector3 marker_pos(marker.position.position.x,marker.position.position.y,marker.position.position.z);
            marker_pos.normalize();

            // the assumption now is that the angle between the marker's position vector (pos) and the
            // normal of the marker (v2, here) always has to be an acute angle (<90), 
            // if it is not, the normal is pointing away from us and we see the "wrong side"
            // consequently, if pos * v2 > 0 we have an acute angle and the quaternion is fine
            // otherwise in pos * v2 < 0, it needs to be flipped, easiest is to just flip v2 and then compute the quaternion
            bool current_marker_is_acute;
            if (marker_pos.dot(axis_vector) > 0) current_marker_is_acute = true;
            else current_marker_is_acute = false;


            // only put in the array the whycon detections that meet the filter criterions specified
            if (useAcuteFilter){
                if (current_marker_is_acute) {
                    if (sqrt(marker.position.position.x*marker.position.position.x + marker.position.position.z*marker.position.position.z) < maxDetectionDistance){
                        // taken from inspired by https://stackoverflow.com/a/11741520. transfrom 
                        // get the quaternion between the up_vector and the reported
                        tf::Vector3 right_vector = axis_vector.cross(up_vector);
                        right_vector.normalized();
                        tf::Quaternion quat(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
                        quat.normalize();
                        geometry_msgs::Quaternion marker_orientation;
                        tf::quaternionTFToMsg(quat, marker_orientation);

                        marker.position.orientation = marker_orientation;

                        // Euler angles
                        marker.rotation.x = objectArray[i].pitch;
                        marker.rotation.y = objectArray[i].roll;
                        marker.rotation.z = objectArray[i].yaw;

                        // Generate RVIZ marker for visualisation
                        visualization_msgs::Marker visualMarker;
                        visualMarker.header = msg->header;
                        // visualMarker.header.stamp = ros::Time();
                        visualMarker.ns = "whycon";
                        visualMarker.id = (identify) ? marker.id : i;
                        visualMarker.type = visualization_msgs::Marker::SPHERE;
                        visualMarker.action = visualization_msgs::Marker::MODIFY;
                        visualMarker.pose = marker.position;
                        visualMarker.scale.x = circleDiameter;  // meters
                        visualMarker.scale.y = circleDiameter;
                        visualMarker.scale.z = 0.01;
                        visualMarker.color.r = 0.0;
                        visualMarker.color.g = 1.0;
                        visualMarker.color.b = 0.0;
                        visualMarker.color.a = 1.0;
                        visualMarker.lifetime = ros::Duration(0.2);  // sec
                        markerArray.markers.push_back(marker);
                        visualArray.markers.push_back(visualMarker);
                    }
                }
            }
            else{
                if (sqrt(marker.position.position.x*marker.position.position.x + marker.position.position.z*marker.position.position.z) < maxDetectionDistance){
                    // taken from inspired by https://stackoverflow.com/a/11741520. transfrom 
                    // get the quaternion between the up_vector and the reported
                    tf::Vector3 right_vector = axis_vector.cross(up_vector);
                    right_vector.normalized();
                    tf::Quaternion quat(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
                    quat.normalize();
                    geometry_msgs::Quaternion marker_orientation;
                    tf::quaternionTFToMsg(quat, marker_orientation);

                    marker.position.orientation = marker_orientation;

                    // Euler angles
                    marker.rotation.x = objectArray[i].pitch;
                    marker.rotation.y = objectArray[i].roll;
                    marker.rotation.z = objectArray[i].yaw;

                    // Generate RVIZ marker for visualisation
                    visualization_msgs::Marker visualMarker;
                    visualMarker.header = msg->header;
                    // visualMarker.header.stamp = ros::Time();
                    visualMarker.ns = "whycon";
                    visualMarker.id = (identify) ? marker.id : i;
                    visualMarker.type = visualization_msgs::Marker::SPHERE;
                    visualMarker.action = visualization_msgs::Marker::MODIFY;
                    visualMarker.pose = marker.position;
                    visualMarker.scale.x = circleDiameter;  // meters
                    visualMarker.scale.y = circleDiameter;
                    visualMarker.scale.z = 0.01;
                    visualMarker.color.r = 0.0;
                    visualMarker.color.g = 1.0;
                    visualMarker.color.b = 0.0;
                    visualMarker.color.a = 1.0;
                    visualMarker.lifetime = ros::Duration(0.2);  // sec
                    markerArray.markers.push_back(marker);
                    visualArray.markers.push_back(visualMarker);
                }
            }

        }
    }

	/*geometry_msgs::Pose start_pose;
	start_pose.position.x = 0.0;
	start_pose.position.y = 0.0;
	start_pose.position.z = 0.1;
	start_pose.orientation.x = 0.0;
	start_pose.orientation.y = 0.0;
	start_pose.orientation.z = 0.0;
	start_pose.orientation.w = 0.0;

	geometry_msgs::Twist start_twist;
	start_twist.linear.x = 0.0;
	start_twist.linear.y = 0.0;
	start_twist.linear.z = 0.0;
	start_twist.angular.x = 0.0;
	start_twist.angular.y = 0.0;
	start_twist.angular.z = 0.0;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = (std::string) "robot";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;
    modelstate.twist = start_twist;*/


    // only publish if there a certain amount of markers detected that meet the conditions
    if(markerArray.markers.size() >= minDetectionsToPublish){
        markers_pub.publish(markerArray);
        visual_pub.publish(visualArray);
    }

    //draw stuff on the GUI 
    if (useGui){
        gui->drawImage(image);
        gui->drawTimeStats(evalTime,numMarkers);
        gui->displayHelp(displayHelp);
        gui->guideCalibration(calibNum,fieldLength,fieldWidth);
    }

    for (int i = 0;i<numMarkers && useGui && drawCoords;i++){
        if (currentSegmentArray[i].valid) {
		gui->drawStats(currentSegmentArray[i].minx-30,currentSegmentArray[i].maxy,objectArray[i],trans->transformType == TRANSFORM_2D);
	}

        if (currentSegmentArray[i].ID != 0) { 
		gui->drawPath(currentSegmentArray[i], drawPath);	
	}

	gazebo_publish(objectArray[i]);
        
    }

    //printf("INDEXGUI: %i %i %i %i\n",indexGUI[0],indexGUI[1],indexGUI[2],indexGUI[3]);
    gui->drawWorkZone(currentSegmentArray[indexGUI[0]].x, currentSegmentArray[indexGUI[0]].y, currentSegmentArray[indexGUI[1]].x, currentSegmentArray[indexGUI[1]].y);
    gui->drawWorkZone(currentSegmentArray[indexGUI[1]].x, currentSegmentArray[indexGUI[1]].y, currentSegmentArray[indexGUI[3]].x, currentSegmentArray[indexGUI[3]].y);
    gui->drawWorkZone(currentSegmentArray[indexGUI[3]].x, currentSegmentArray[indexGUI[3]].y, currentSegmentArray[indexGUI[2]].x, currentSegmentArray[indexGUI[2]].y);
    gui->drawWorkZone(currentSegmentArray[indexGUI[2]].x, currentSegmentArray[indexGUI[2]].y, currentSegmentArray[indexGUI[0]].x, currentSegmentArray[indexGUI[0]].y);

    /*float x1 = -0.73;
    float y1 = 1.0;
    float z1 = 1.8;

    float x2 = -0.73;
    float y2 = -0.7;
    float z2 = 1.8;

    std::vector<float> xy1 = trans->reTransformXYchris(&x1, &y1, &z1);
    std::vector<float> xy2 = trans->reTransformXYchris(&x2, &y2, &z2);
    printf("xy1 = %f, %f, %f\n", xy1[0], xy1[1], xy1[2]);
    printf("xy2 = %f, %f, %f\n", xy2[0], xy2[1], xy2[2]);

    gui->drawLine(xy1[0],xy1[1],xy2[0],xy2[1],1);*/

    //establishing the coordinate system by manual or autocalibration
    if (autocalibrate && numFound == numMarkers) autocalibration();
    if (calibNum < 4) manualcalibration();

    /* empty for-cycle that isn't used even in master orig version
       for (int i = 0;i<numMarkers;i++){
    //if (currentSegmentArray[i].valid) printf("Object %i %03f %03f %03f %03f %03f\n",i,objectArray[i].x,objectArray[i].y,objectArray[i].z,objectArray[i].error,objectArray[i].esterror);
    }*/

    //gui->saveScreen(runs);
    if (useGui) gui->update();
    if (useGui) processKeys();
}

void CWhycon::gazebo_publish(STrackedObject object){
	printf("Gazebo object position.x: %f \n", object.x);
	printf("Gazebo object position.y: %f \n\n", object.y);
	printf("Gazebo object ID: %i \n", object.ID);

	gazebo_msgs::ModelState modelstate;

	if (object.ID == 0){
		if (object.x > -0.1 && object.x < 0.1 && object.y > -0.1 && object.y < 0.1) modelstate.model_name = "V00";
		else if (object.x > 0.9 && object.x < 1.1 && object.y > -0.1 && object.y < 0.1) modelstate.model_name = "VX0";
		else if (object.x > -0.1 && object.x < 0.1 && object.y > 0.9 && object.y < 1.1) modelstate.model_name = "V0Y";
		else if (object.x > 0.9 && object.x < 1.1 && object.y > 0.9 && object.y < 1.1) modelstate.model_name = "VXY";
	} else {
		if (object.ID == 2) modelstate.model_name = "robot_2";
		else if (object.ID == 4) modelstate.model_name = "robot_4";
		else if (object.ID == 6) modelstate.model_name = "robot_6";
		else if (object.ID == 8) modelstate.model_name = "robot_8";
	}

	geometry_msgs::Pose start_pose;
	start_pose.position.x = object.x;
	start_pose.position.y = object.y;
	start_pose.position.z = 0.0;
	start_pose.orientation.x = 0.0;
	start_pose.orientation.y = 0.0;
	start_pose.orientation.z = 0.0;
	start_pose.orientation.w = 0.0;

	geometry_msgs::Twist start_twist;
	start_twist.linear.x = 0.0;
	start_twist.linear.y = 0.0;
	start_twist.linear.z = 0.0;
	start_twist.angular.x = 0.0;
	start_twist.angular.y = 0.0;
	start_twist.angular.z = 0.0;

	
	//modelstate.model_name = (std::string) std::to_string(object.ID);
	modelstate.reference_frame = (std::string) "world";
	modelstate.pose = start_pose;
	modelstate.twist = start_twist;

	gazebo_pub.publish(modelstate);
}

// dynamic parameter reconfiguration
void CWhycon::reconfigureCallback(CWhycon *whycon, whycon_ros::whyconConfig& config, uint32_t level){
    ROS_INFO("[Reconfigure Request]\n"
            "numMarkers %d circleDiam %lf identify %d\n"
            "initCircularityTolerance %lf finalCircularityTolerance %lf\n"
            "areaRatioTolerance %lf centerDistTolerance %lf centerDistToleranceAbs %lf\n",
            config.numMarkers, config.circleDiameter, config.identify,\
            config.initialCircularityTolerance, config.finalCircularityTolerance,\
            config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);

    whycon->numMarkers = (config.numMarkers > whycon->maxMarkers) ? whycon->maxMarkers : config.numMarkers;
    whycon->fieldLength = config.fieldLength;
    whycon->fieldWidth = config.fieldWidth;
    whycon->angleWorkZone = config.angleWorkZone;
    whycon->dM0 = config.dM0;
    whycon->dM1 = config.dM1;
    whycon->identify = config.identify;
    whycon->circleDiameter = config.circleDiameter / 100.0;

    whycon->trans->reconfigure(config.circleDiameter);

    for (int i = 0;i<whycon->maxMarkers;i++) whycon->detectorArray[i]->reconfigure(\
            config.initialCircularityTolerance, config.finalCircularityTolerance,\
            config.areaRatioTolerance,config.centerDistanceToleranceRatio,\
            config.centerDistanceToleranceAbs, config.identify, config.minSize);
}

// cleaning up
CWhycon::~CWhycon(){
    ROS_DEBUG("Releasing memory.");
    free(calibTmp);
    free(objectArray);
    free(objectArrayPath);
    free(currInnerSegArray);
    free(currentSegmentArray);
    free(currentSegmentArrayPath);
    free(lastSegmentArray);

    delete image;
    if (useGui) delete gui;
    for (int i = 0;i<maxMarkers;i++) delete detectorArray[i];
    free(detectorArray);
    delete trans;
    delete decoder;
    delete n;
}

CWhycon::CWhycon(){
    imageWidth = 640;
    imageHeight = 480;
    circleDiameter = 0.122;
    fieldLength = 1.00;
    fieldWidth = 1.00;
    angleWorkZone = 0.00;
    dM0 = 0.00;
    dM1 = 0.00;

    identify = false;
    numMarkers = 0;
    numFound = 0;
    numStatic = 0;

    idBits = 0;
    idSamples = 360;
    hammingDist = 1;

    stop = false;
    moveVal = 1;
    moveOne = moveVal; 
    useGui = true;
    guiScale = 1;
    keyNumber = 10000;
    keys = NULL;
    displayHelp = false;
    drawCoords = true;
    drawPath = false;
    runs = 0;
    evalTime = 0;
    screenWidth= 1920;
    screenHeight = 1080;

    calibNum = 5;
    calibTmp = (STrackedObject*) malloc(calibrationSteps * sizeof(STrackedObject));
    calibStep = calibrationSteps+2;
    autocalibrate = false;
    lastTransformType = TRANSFORM_2D;
    wasMarkers = 1;

    useAcuteFilter = false;
    maxDetectionDistance = 100;
    minDetectionsToPublish = 1;

}

void CWhycon::init(char *fPath, char *calPath){
    n = new ros::NodeHandle("~");
    image_transport::ImageTransport it(*n);
    image = new CRawImage(imageWidth,imageHeight, 3);

    // loading params and args from launch file
    fontPath = fPath;
    calibDefPath = calPath;
    n->param("useGui", useGui, true);
    n->param("idBits", idBits, 3);
    n->param("idSamples", idSamples, 360);
    n->param("hammingDist", hammingDist, 1);
    n->param("maxMarkers", maxMarkers, 100);
    n->param("useAcuteFilter", useAcuteFilter, false); //only whycons within an acute angle repect the camera will be published
    n->param("maxDetectionDistance",maxDetectionDistance,100); // whycon futher away than this distance won't be published
    n->param("minDetectionsToPublish", minDetectionsToPublish, 1); //minimum amount of detected fiducial before start publishing

    moveOne = moveVal;
    moveOne  = 0;

    objectArray = (STrackedObject*) malloc(maxMarkers * sizeof(STrackedObject));
    objectArrayPath = (STrackedObject*) malloc(maxMarkers * sizeof(STrackedObject));
    currInnerSegArray = (SSegment*) malloc(maxMarkers * sizeof(SSegment));
    currentSegmentArray = (SSegment*) malloc(maxMarkers * sizeof(SSegment));
    currentSegmentArrayPath = (SSegment*) malloc(maxMarkers * sizeof(SSegment));
    lastSegmentArray = (SSegment*) malloc(maxMarkers * sizeof(SSegment));

    // determine gui size so that it fits the screen
    while (imageHeight/guiScale > screenHeight || imageHeight/guiScale > screenWidth) guiScale = guiScale*2;

    // initialize GUI, image structures, coordinate transformation modules
    if (useGui) gui = new CGui(imageWidth,imageHeight,guiScale, fontPath.c_str());
    trans = new CTransformation(imageWidth,imageHeight,circleDiameter, calibDefPath.c_str());
    trans->transformType = TRANSFORM_NONE;		//in our case, 2D is the default

    detectorArray = (CCircleDetect**) malloc(maxMarkers * sizeof(CCircleDetect*));

    // initialize the circle detectors - each circle has its own detector instance 
    for (int i = 0;i<maxMarkers;i++) detectorArray[i] = new CCircleDetect(imageWidth,imageHeight,identify, idBits, idSamples, hammingDist);
    image->getSaveNumber();

    decoder = new CNecklace(idBits,idSamples,hammingDist);

    // initialize dynamic reconfiguration feedback
    dynSer = boost::bind(&CWhycon::reconfigureCallback, this, _1, _2);
    server.setCallback(dynSer);

    // subscribe to camera topic, publish topis with card position, rotation and ID
    subInfo = n->subscribe("/camera/camera_info", 1, &CWhycon::cameraInfoCallback, this);
    subImg = it.subscribe("/camera/image_raw", 1, &CWhycon::imageCallback, this);
    markers_pub = n->advertise<whycon_ros::MarkerArray>("/whycon_ros/markers", 1);
    visual_pub = n->advertise<visualization_msgs::MarkerArray>( "/whycon_ros/visual", 0 );
    gazebo_pub = n->advertise<gazebo_msgs::ModelState>( "/gazebo/set_model_state", 10 );

    while (ros::ok()){
        ros::spinOnce();
        usleep(30000);
        if(stop) break;
    }
}

int main(int argc,char* argv[]){
    ros::init(argc, argv, "whycon_ros");

    CWhycon *whycon = new CWhycon();
    whycon->init(argv[1], argv[2]);

    delete whycon;

    return 0;
}
