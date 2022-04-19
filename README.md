# 6MWT
Six minute walk test for pulmonary rehabilitation

System working on Linux Distribution, dedicate to work as "feature" on ECG Device. 
Physically there are two devices - transmitter and receiver. 
Patient walks in circles in the hall wearing transmitter. 
Two devices are communicate with each other using internal radio protocol. 
Receiver is connected to ECG via USB cable and respond after request on UART transmission. 
Host request distance data every 100 ms. 
Colected raw samples are filtered 'online' using moving average. 
Using difference between distance on previous request and actual, program can calculate walked distance. 
On filtred samples is working alghoritm, which detect extremas. 
Extremum is important information about change directory of walk (patient turned back, and go in opposite direction). 

Example plot below, "cross" mean detected extrema
![plot](https://user-images.githubusercontent.com/87012097/163970487-1ce7266b-8ff2-46bf-aaed-aa54160b078f.png)


Compiler gcc 4.8
