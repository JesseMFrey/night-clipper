

FQBN=esp32:esp32:adafruit_qtpy_esp32s3_nopsram 
SERIAL_DEV:=/dev/ttyACM0

BUILD_DIR  := ./build 

SRC        := $(wildcard *.ino)
HDRS       := $(wildcard *.h)
BIN        := $(BUILD_DIR)/$(SRC).bin
ELF        := $(BUILD_DIR)/$(SRC).elf

.PHONY: all
all: $(ELF) 

.PHONY: $(ELF)
$(ELF): $(SRC) $(HDRS)
	arduino-cli compile --build-cache-path $(BUILD_DIR) -b $(FQBN)

.PHONY: upload
upload: 
	@echo "---> Uploading code"
	arduino-cli upload -b $(FQBN) -p $(SERIAL_DEV)


.PHONY: clean
clean:
	@echo "---> Cleaning the build directory"
	rm -rf build
