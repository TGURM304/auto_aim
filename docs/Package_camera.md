# Package_Document - camera

### 1、节点

| 名称        | 功能              | 其他                               |
| ----------- | ----------------- | ---------------------------------- |
| stream_node | 发布BGR8图像消息  | topic_name: **camera/stream**      |
| gray_node   | 发布MONO8图像消息 | topic_name: **camera/gray_stream** |
| test_node   | 工业相机测试节点  | 由官方demo修改而来                 |