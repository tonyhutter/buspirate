obj-m += buspirate.o

# Point this to your Linux kernel source directory
KDIR:=/usr/src/linux

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean
