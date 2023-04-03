SHELL=cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
OBJ = controller.o
PORTN=$(shell type COMPORT.inc)

controller.elf: $(OBJ)
	$(CC) $(ARCH) -o controller.elf controller.o -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=controller.map
	$(OBJCPY) controller.elf
	@echo Success!
   
controller.o: controller.c
	$(CC) -mips16 -g -x c -c $(ARCH) -MMD -o controller.o controller.c -DXPRJ_default=default -legacy-libc

clean:
	@del *.o *.elf *.hex *.d *.map 2>NUL
	
LoadFlash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	pro32.exe -p controller.hex
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

dummy: controller.hex controller.map
	$(CC) --version

explorer:
	@explorer .