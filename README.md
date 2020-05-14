# Self Collect Machine

This project is about to design a self collecting system which ease the customer 
collect thier purchased items during office or after office hours -- by scanning
the code (BAR/QR)

**Notes**

StoreQR: OrderID & CustomerEmail
CustQR: OrderID

### Required Package:
1. Python barcode decoding library -- Zbar
	1. sudo apt-get install libzbar0
	2. pip install pyzbar

2. UbiquityRobotics/raspicam_node
	1. https://github.com/UbiquityRobotics/raspicam_node

3. cv_camera
	1. http://wiki.ros.org/cv_camera

## Scripts
### Using USB Camera
1. camera_preview.py
> Additional useful to test and calibrate a camera

2. camera_barcode_recognition.py
> Publications: 
> * /rosout [rosgraph_msgs/Log]
> * /scanned_barcode [std_msgs/String]
>
> Subscriptions: 
> * /cv_camera/camera_info [sensor_msgs/CameraInfo]
> * /cv_camera/image_raw [sensor_msgs/Image]
> * /scan_status [std_msgs/String]

3. camera_barcode_record.py --> simplified version barcode_record.py

### Using Raspberry Pi Camera (Raspicam)
1. raspicam_preview.py

2. raspicam_barcode_recognition.py

3. raspicam_barcode_record.py --> simplified version barcode_record.py

### Barcode
1. barcode_identification.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
>  * /scan_mode [std_msgs/String]
> Subscriptions: 
>  * /scanned_barcode [std_msgs/String]

2. customer_barcode_record.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
>  * /scan_status [std_msgs/String]
> Subscriptions: 
>  * /scan_mode [std_msgs/String]
>  * /scanned_barcode [std_msgs/String]

3. store_barcode_record.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
>  * /scan_status [std_msgs/String]
> Subscriptions: 
>  * /scan_mode [std_msgs/String]
>  * /scanned_barcode [std_msgs/String]

4. customer_barcode_validate.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
> Subscriptions: 
>  * /scan_mode [std_msgs/String]
>  * /scanned_barcode [std_msgs/String]

5. barcode_record.py
6. barcode_validate.py

## Launch
1. camera_robot.launch
2. raspicam_robot.launch

## How to troubleshoot the project (need to run by following order)
### USB Camera
#### camera_robot.launch
1. roslaunch self_collect_machine camera_robot.launch
> This will run the camera -- in ready mode
> To test either the camera is showing a sequence of images; video run:
> rosrun self_collect_machine camera_preview.py

**Updated**

#### camera_barcode_recognition.py
1. rosrun self_collect_machine camera_barcode_recognition.py
> This will run the code recognition to recognize a Bar-Code or QR-Code.

#### barcode_identification.py
1. rosrun self_collect_machine barcode_identification.py
> This will identify the Machine Mode -- Customer or Store -- with the 
> smart-tags under the Bar-Code or QR-Code eg: Customer Code only consist of 
> OrderID only while in Store Code consist of OrderID and CustomerEmail

#### store_barcode_record.py
1. rosrun self_collect_machine store_barcode_record.py
> This will record the bind together both OrderID and CustomerEmail with the
> BoxID based on the availability. Once binded, the datas will be recorded
> under the store_barcode.csv

#### Microcontroller: 
1. Download the ino file into Arduino Mega 
> sensor reading on the availability of the box; 1 is empty and 0 is full (occupied).

#### customer_barcode_record.py
1. rosrun self_collect_machine customer_barcode_record.py
> This will record each OrderID scanned under the customer_barcode.csv and 
> simultaneously checking with the store_barcode.csv to allow the system to 
> open designated BoxID
