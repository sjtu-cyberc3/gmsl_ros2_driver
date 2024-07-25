# 基于米文动力SDK的GMSL相机ROS2驱动

## 节点参数

* out_fmt: 图像像素格式（UYVY，BGRA32）

* out_w: 图像宽度

* out_h: 图像高度

* camera_num: 相机数量

## 发布话题

* video[n]/raw/image: 其中n为第n个相机，对应/dev/video[n]处的设备。

建议通过设置namespace为消息名增加前缀。例如将namespace设置为/driver/gmsl，则话题变为/driver/gmsl/video[n]/raw/image。
