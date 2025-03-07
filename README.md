# ROS Socket Transfer 
This package allows you to communicate topics/services/actions between different devices by creating direct socket communication. 

## Why?
While ROS2 does offer a similar thing out of the box, you will quickly find the limitations of that solution upon trying to use it for anything other than test cases. For example, trying to move a stream of images (or point clouds) will make the entire communication stop working. For such cases, manually managing the communication through a UDP or TCP socket is a much better solution.

 ## How?
 For any topic/service/action that you want to transfer between two devices, you will need to create a pair of nodes: a client (sends the data) and a server (receives the data). The client will consume the local ROS communication and send it through the socket. The server will receive it at the other end and create a new ROS message with the same data that it will publish locally. For services and actions, the server is also responsible for returning the corresponding feedback back to the client.

 This package contains the infrastructure for all this to seamlessly happen so the rest of your nodes don't need to know that there is any inter-device communication happening at all. However, in order to be able to use this you will need to provide the logic for serializing/deserializing any specific type of message you are interested on transfering. This is done by creating you own node which provides an specialization of the `Serializer<Msg>` template.
 
 See [these packages](https://github.com/MAPIRlab/utils/tree/ros2/Communications/sockets) for examples.

 ## Params and Usage

 TO-DO
