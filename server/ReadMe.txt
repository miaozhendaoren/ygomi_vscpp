This project is for Newco In-Server Software demo.

Develop environments:
Visual Studio 2010 Ultimate VC++, OpenCV2.4.10, OpenGL1.1, glut3.70beta.
Operation System:
Windows 7 Professional 32-bit/64-bit
Backgroud: 
use VC to do data process, OpenGL to implement sample 3D engine, glut to show in a window.
instead of MFC, because MFC + WGL is only supported in Windows, glut can port to other operation system easily.

This project is including the following threads:
1. main thread, it will be used for 3D visualization.
2. receive thread to receiving the detected different image message from vehicle, Including receive message and put the message to message queue.
3. DB accesss thread to operating the Digital Horizon Data Difference collector and notice the update the thread.
4. update thread to send the update message to vehicle.
5. visualize thread to process the data need in 3D visualization

Files structure:
server\include:   basic type define
server\resource:  basic pictures, digital horizon database file
server\config  :  basic configure, include server ip and port, inVehicle ip and port.    
server\sources\frameWork:     the thread information and system arct code
server\sources\interface:     basic communication class
server\sources\visualization: sample 3D engine code. 
server\sources\database :     Digital Horizon Data method code.
    
