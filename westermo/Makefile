CURR_DIR = $(shell pwd)

BIN_TAG = $(shell git describe --abbrev=0 2> /dev/null)

UTS_RELEASE = $(shell git describe --always 2> /dev/null)

.PHONY: all clean dagger utsrelease

utsrelease:
	@echo "Generating utsrelease.h"
	@echo '#define UTS_RELEASE "$(UTS_RELEASE)"' > utsrelease.h
	 
dagger: utsrelease
	@echo "Making binary.0 for Dagger tag $(BIN_TAG)"
	cd .. && ./myclean.pl && ./build.pl -b ac3_customer0 -f spi -v $(BIN_TAG) -i spi -c -o $(CURR_DIR)/$(BIN_TAG)
	@echo "Removing config bytes in binary for barebox"
	tail -c +13 $(BIN_TAG)/bin_hdr/bin_hdr.bin > binary.0
	
all: dagger

clean:
	cd .. && ./myclean.pl
	