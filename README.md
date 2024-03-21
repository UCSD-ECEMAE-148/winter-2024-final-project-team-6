# Team 6 Final Project Drone Following

**Objectives**
- Use OpenCV to recognize drone
- Integrate it into the Jetson and follow the Drone while navigating
- Reducing latencey for optimized relay times and rapid tracking

### Absratact 
This teams final project was a nold attempt at solving one of the most potent problems faced in the tracking and recovery sector of aerial vehicles namely drones which can be used in delivering courier packages to remote locations. Using line detection in a 2-dimentional setting is already challenging but this project in many was pushed the limit of the Nano and Oakd's processing capability by utilizing Roboflow trained models to succesfully identify the airborne drone.

## The Minimum Viable Product
- Identify the drone
- Use ROS2 as a platform to track the drone
- Integrate a platform that can be used for landing the drone
- Use effective algorithms to ensure that our erroe values allow us to keep the drone in frame while it is moving.

## What We Could Have Done If Had More Time.
- Incorporate a second camera to land the drone perfectley on the landing base
- Incorporate obstruction avoidance using Lidar or RGB depth
- add a solid landing base for the drone to land on the car.
  

## Our Project In A Nut Shell






## Hardware 

  <p align="center"><img src="https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-6/assets/164306890/d3f91601-d6c6-426f-b8aa-4733af1c4f2f" width="700" height="400"></p>


As you can see that one of the main focus was to design a camera mount with an adjustable angle of attack for us to optimize the cars feild of view and take in account that latencey would result in the drone being lost intermintentley. to account for this we tried to optimize our field of view.

  <p align="center"><img src="https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-6/assets/164306890/fec69d6d-94ca-4cc7-9c39-1b82783bdb7e" width="700" height="400"></p>



## The Roboflow Trained Model


Using Roboflow we were able to train the model and push it on to our UCSD docker container.
importing the roboflow model was one thing, using code to located the centroid and filter our confidence level to an optimal value such that our Oak'd would not mistake any other foriegn object was another hassle.As can be seen in the images below:

  <p align="center"><img src="https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-6/assets/164306890/3cf85766-3ce2-4085-9c80-a0b58e8be5d9"width="400" height="400"></p>



The image below illustrates the optimized version and the code utilized in attaining centriod coordinates with respective error bars. Now the object with the highest confidence interval is selected for centroid placement and this holds valid for our objective that is to track the drone.

            predictions = result["predictions"]

            # log the x y coords of the drone
            self.prev_highest_confidence = 0
            for p in predictions:
                
                self.confidence = float(p.confidence)
                if self.confidence > self.prev_highest_confidence:
                    self.prev_highest_confidence = self.confidence
                    self._x = int(p.x)
                    self._y = int(p.y)

                self.get_logger().info(f'x: {self._x}, y: {self._y}, Confidence: {self.confidence}, Highest confidence: {self.prev_highest_confidence}')

            # Get the center coordinates of the frame
            #if 
            frame_height, frame_width, num_channels = frame.shape
            center_x = frame_width // 2
            center_y = frame_height // 2


            ### CALCULATE THE ERROR FOR STEERING ###
            self.error = self._x - center_x
            self.get_logger().info(f'Error: {self.error}')

  
   <p align="center"><img src="https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-6/assets/164306890/4158c6bd-3b4d-49e4-834d-327ca3fca48a"width="400" height="400"></p>


## Using ROS2 To Configure The Respective Nodes

Once we trained our model the next step was to create a Python Package in ROS2 in the SRC directory to configure our nodes to publish and subrcribe to the displavyed information by the Oak'd camera.

**Our Nodes Can Be Divided into the following Categories:**
- 












