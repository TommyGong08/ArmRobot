# qrcode_ros
QRコードを読み取るパッケージ
![](/img/readme.png)






QRコードの生成はここのサイトがおすすめ
[QR Code Generator](https://ja.qr-code-generator.com/)

---
## Install
```
# clone this repository
$ cd ~/catkin_ws/src/
$ git clone https://git.......
$ cd .. & catkin_make

# install pyzbar & zbar
$ sudo pip install pyzbar
$ sudo apt install libzbar0
```

---
## Usage
```
$ roslaunch qrcode_ros decoder.launch
```

---
## Publication
- "/qrcode_ros/decode_data" [std_msgs/String]

---
## Subscription
- "/camera/rgb/image_raw" [sensor_msgs/Image]

---
## Parameters
- "/qrcode_ros/decode_ctrl" → decode QRcode or Not [default:True]
- "/qrcode_ros/sub_img_name" → name of topic(sensor_msgs/Image) to subscribe [default:"/camera/rgb/image_raw"]
