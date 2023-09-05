# yolo_traffic_light
Detection and Recognition Traffic Lights

## Infromation
* Yolo version : Gaussian YOLOv3
* Dataset : BDD Dataset

## Requirements
1. Autoware-AI (vision_darknet_detect)
2. BDD Dataset
3. Camera or Video
  
## How to work
1. Detection of traffic lights using Gaussian YOLO  
   <img src=https://github.com/simfrog/yolo_traffic_light/assets/31130917/f5c80e85-e79b-4084-a16a-c6ebab09ecc5.png>
   
2. Select the largest of the detected traffic lights and extract only the traffic lights  
   <img src=https://github.com/simfrog/yolo_traffic_light/assets/31130917/d0c7745c-c270-45de-9453-edfb545880cd.png width=100, height=30>

3. Perform binarization to convert the light part to black and the rest to white  
   <img src=https://github.com/simfrog/yolo_traffic_light/assets/31130917/5a224c90-2731-4b80-8b91-745acdd83232.png width=100, height=50>

4. After divide between three and four traffic lights, calculate pixels, and distinguish signals according to the location of the black pixels

## How to run
#### rosrun yolo_traffic_light yolo_traffic_light.cpp
