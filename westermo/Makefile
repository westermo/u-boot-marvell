CROSS_COMPILE=arm-msys-linux-gnueabi-
CROSS_COMPILE_BH=arm-msys-linux-gnueabi-
BIN_TAG=2018.10.0
CURR_DIR = $(shell pwd)

UTS_RELEASE = $(shell git describe --always 2> /dev/null)

.PHONY: all clean dagger utsrelease

utsrelease:
	@echo "Generating utsrelease.h"
	@echo '#define UTS_RELEASE "$(UTS_RELEASE)"' > utsrelease.h
	 
dagger: utsrelease
	@echo "Making binary.0 for Dagger"
	cd .. && ./myclean.pl && ./build.pl -b ac3_customer0 -f spi -v $(BIN_TAG) -i spi -c -o $(CURR_DIR)/$(BIN_TAG)
	@echo "Removing config bytes in binary for barebox"
	tail -c +13 $(BIN_TAG)/bin_hdr/bin_hdr.bin > binary.0
	
all: dagger

clean:
	cd .. && ./myclean.pl
	