
BOARD?=esp32:esp32:esp32c3:CDCOnBoot=cdc

PORT?=COM11

FW_PATH?=fw
BUILD = build

.PHONY: default lint build flash clean

default: lint build flash clean


lint:
	cpplint --filter=-readability/casting --extensions=ino  ${FW_PATH}\fw.ino

build:
	arduino-cli compile --fqbn $(BOARD) ${FW_PATH} --output-dir ${FW_PATH}\${BUILD}
	

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) ${FW_PATH}

clean:
	del ${FW_PATH}\build /s /q


# To run the makefile, open the terminal and type:
# make lint build flash clean
# or
# make - to run everything
# Passing arguments:
# make PORT=COM3