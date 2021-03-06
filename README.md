# Self Collect Machine

This project is about to design a self collecting system which ease the customer 
collect thier purchased items during office or after office hours -- by scanning
the code (BAR/QR)

**Updated**
The project has been divided into several part to simplified the total system 
which is:
1. accessing_raspicam_usbcam -- https://github.com/KhairulIzwan/accessing_raspicam_usbcam.git
2. common_barcode_application -- https://github.com/KhairulIzwan/common_barcode_application.git
3. common_i2c_application -- https://github.com/KhairulIzwan/common_i2c_application.git
4. common_spi_application -- https://github.com/KhairulIzwan/common_spi_application.git
5. common_video-recording_application -- https://github.com/KhairulIzwan/common_video-recording_application

**Usage**
1. roslaunch self_collect_machine run.launch
2. rosrun common_video-recording_application time_record.py

**Notes**

StoreQR: OrderID & CustomerEmail

CustQR: OrderID

### Required Package:
#### Python barcode decoding library -- Zbar
1. sudo apt-get install libzbar0
2. pip install pyzbar

#### UbiquityRobotics/raspicam_node
1. https://github.com/UbiquityRobotics/raspicam_node

#### cv_camera
1. http://wiki.ros.org/cv_camera

## Scripts
### Using USB Camera
#### camera_preview.py
> Additional useful to test and calibrate a camera

#### camera_barcode_recognition.py
> Publications: 
> * /rosout [rosgraph_msgs/Log]
> * /scanned_barcode [std_msgs/String]
>
> Subscriptions: 
> * /cv_camera/camera_info [sensor_msgs/CameraInfo]
> * /cv_camera/image_raw [sensor_msgs/Image]
> * /scan_status [std_msgs/String]

#### camera_barcode_record.py --> simplified version barcode_record.py

### Using Raspberry Pi Camera (Raspicam)
#### raspicam_preview.py

#### raspicam_barcode_recognition.py

#### raspicam_barcode_record.py --> simplified version barcode_record.py

### Barcode
#### barcode_identification.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
>  * /scan_mode [std_msgs/String]
> Subscriptions: 
>  * /scanned_barcode [std_msgs/String]

#### customer_barcode_record.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
>  * /scan_status [std_msgs/String]
> Subscriptions: 
>  * /scan_mode [std_msgs/String]
>  * /scanned_barcode [std_msgs/String]

#### store_barcode_record.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
>  * /scan_status [std_msgs/String]
> Subscriptions: 
>  * /scan_mode [std_msgs/String]
>  * /scanned_barcode [std_msgs/String]

#### customer_barcode_validate.py
> Publications: 
>  * /rosout [rosgraph_msgs/Log]
> Subscriptions: 
>  * /scan_mode [std_msgs/String]
>  * /scanned_barcode [std_msgs/String]

#### barcode_record.py
#### barcode_validate.py

**Updated**
Some code are updated and name ended with *rev1* or *rev2*

## Launch
1. camera_robot.launch
2. raspicam_robot.launch

## How to troubleshoot the project (need to run by following order)
#### Camera
roslaunch self_collect_machine camera_robot.launch
> This will run the camera -- in ready mode
> To test either the camera is showing a sequence of images; video run:
> rosrun self_collect_machine camera_preview.py

#### Barcode Recognition
rosrun self_collect_machine camera_barcode_recognition.py
> This will run the code recognition to recognize a Bar-Code or QR-Code.

#### Barcode Identification
rosrun self_collect_machine barcode_identification.py
> This will identify the Machine Mode -- Customer or Store -- with the 
> smart-tags under the Bar-Code or QR-Code eg: Customer Code only consist of 
> OrderID only while in Store Code consist of OrderID and CustomerEmail

#### Store Barcode Record
rosrun self_collect_machine store_barcode_record.py
> This will record the bind together both OrderID and CustomerEmail with the
> BoxID based on the availability. Once binded, the datas will be recorded
> under the store_barcode.csv

#### Microcontroller 
Download the ino file into Arduino Mega 
> sensor reading on the availability of the box; 1 is empty and 0 is full (occupied).

#### Customer Barcode Record
rosrun self_collect_machine customer_barcode_record.py
> This will record each OrderID scanned under the customer_barcode.csv and 
> simultaneously checking with the store_barcode.csv to allow the system to 
> open designated BoxID

## Simplified Version
roslaunch self_collect_machine camera_self_collect_machine_rev1.launch

**OR**

roslaunch self_collect_machine raspicam_self_collect_machine_rev1.launch
