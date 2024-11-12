# Package_Document - camera

### 1、节点

| 名称          | 功能                      | 其他                                                                         |
| ------------- | ------------------------- | ---------------------------------------------------------------------------- |
| stream_node   | 发布相机BGR8图像消息      | topic_name: **camera/stream**                                                |
| video_node    | 发布视频BGR8图像消息      | topic_name: **camera/stream**（为了方便模拟摄像头进行测试，使用了同名topic） |
| ~~gray_node~~ | ~~发布相机MONO8图像消息~~ | ~~topic_name: **camera/gray_stream**~~（弃用此）                     |
| test_node     | 工业相机测试节点          | 由官方demo修改而来                                                           |
