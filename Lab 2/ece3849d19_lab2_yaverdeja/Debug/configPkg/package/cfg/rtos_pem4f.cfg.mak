# invoke SourceDir generated makefile for rtos.pem4f
rtos.pem4f: .libraries,rtos.pem4f
.libraries,rtos.pem4f: package/cfg/rtos_pem4f.xdl
	$(MAKE) -f C:\Users\YAVERD~1\Desktop\ECE384~1\ece3849d19_lab2_yaverdeja/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\YAVERD~1\Desktop\ECE384~1\ece3849d19_lab2_yaverdeja/src/makefile.libs clean

