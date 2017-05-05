-In main, need to include usb_main.h, usb_main.c, R2Protocol.h, R2Protocol.c
-Look at includes in main.c for reference


-usb_main.c contains ProcessIO(), which takes in a data packet struct and puts in
the usb transmission information into the struct. Returns 1 if successfully put into data packet,
and returns 0 if not. Look at R2Protocol.h for struct format.
ProcessIO() gets the usb transmission all at once, or builds it from multiple transmissions,
and then calls R2ProtocolDecode, which is the function that puts the transmission into the struct.



Possible issues: 
-For building transmission from multiple transmissions, sometimes the partialTransLength and transLength variables in ProcessIO() that are set outside of the while loop (for the state machine) get reset to zero, even though they should only be set to zero once every time ProcessIO is called. Possibly due to the while(1) loop in main calling ProcessIO again too quickly before the first ProcessIO could finish executing. I believe I fixed this issue by putting a 150ms delay at the end of ProcessIO. This delay could be a problem if fast repeated transmissions are necessary, but for most transmissions, should be fast enough. 
-Partial transmission building was tested with 100ms delay in between transmission packets sent from a python script. When sending full transmissions, ProcessIO was able to get the entire transmission at once every time, so the partial transmission building was never used. For longer transmissions with large data, partial transmission building may be needed. 



