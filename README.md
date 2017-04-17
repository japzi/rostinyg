## rostinyg
rostinyg is a simple ros package to publish position information of tinyg controller: https://github.com/synthetos/TinyG/

rostinyg communicates with serial port json server: https://github.com/chilipeppr/serial-port-json-server

roslaunch file for rostinyg
```
<launch>
  <node pkg="rostinyg" type="rostinyg" name="rostinyg_nodlet" args="_serial_port_json_server_url:=http://localhost:8989/ws _usb_port:=/dev/ttyUSB0 _report_delay:=100" >
  </node>
</launch>
```
rostinyg publishes position message
- posX (float)
- posY (float)
- posZ (float)
- timestamp (ros::time)

rostinyg uses following libraries with thanks:
- Websocket C++ library: https://github.com/zaphoyd/websocketpp (BSD License)
- JSON C++ library: https://github.com/nlohmann/json (MIT License)
  
