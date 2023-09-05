## qr_detector
The QR codes detector based on zbar library (http://zbar.sourceforge.net), dedicated to ROS systems.

Installation of zbar library on Ubuntu
`sudo apt install libzbar-dev`

Subscribes:
- **/image** (sensor_msgs/Image) - the topic with RGB images which contains QR codes.

Publishes:
- **/qr_codes** (std_msgs/String) - message from each detected QR code is published as a string.