
This project is for Newco In-Vehicle Software demo.

Develop environments:
Visual Studio 2010 Ultimate VC++, OpenCV2.4.10, OpenGL1.1, glut3.70beta.
Operation System:
Windows 7 Professional 32-bit/64-bit
Backgroud: 
use VC to do data process, OpenGL to implement sample 3D engine, glut to show in a window.
instead of MFC, because MFC + WGL is only supported in Windows, glut can port to other operation system easily.

This project includes the following threads:
1. main thread, it will be used for 3D visualisation.
2. thread for receive GPS data using UDP socket.
   it includes Sensor Data Collector module, in the demo, only have GPS data.
3. thread for traffic sign detection. for the demo, we put the following modules into one thread, 
in the product, the Digital Horizon Data Difference Collector maybe a seperate thread, it can cache some datas, and in some condition, it will send Difference data to server side.
   it includes the following modules:
       Image Sensor Data Collector
       Sensor Data Processing Unit
       Difference detection Unit
       Digital Horizon Data Difference Collector
       send Difference data to server
4. thread for receive Server update message
   it includes the following modules:
       Data communication Interface
       Digital Horizon Data Update Manager
5. thread for preproce visualization data, it will call digital horizon data access manager to get the data. 
   it includes Digital Horizon Data Access Manager modules(look ahead)
   

Files structure:
inVehicle\include:   basic type define
inVehicle\resource:  basic pictures, svm files, car model and digital horizon database file
inVehicle\config  :  basic configure, include server ip and port, inVehicle ip and port, vehicle ID, receive GPS port.    
inVehicle\sources\frameWork:     the thread information and system arct code
inVehicle\sources\interface:     receive GPS and basic communication class
inVehicle\sources\visualization: sample 3D engine code. 
inVehicle\sources\database :     Digital Horizon Data method code.
inVehicle\sources\detectImage:   image recogninition code.
   

  