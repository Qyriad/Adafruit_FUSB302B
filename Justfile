# vim: noexpandtab shiftwidth=4
#SERIAL_DEV := "/dev/ttyACM1"
SERIAL_DEV := "/dev/ttyACM2"
#BOARD := "adafruit:samd:adafruit_feather_m4_can"
BOARD := "arduino:mbed_nano:nano33ble"
#SKETCH := "examples/sinkdemo/sinkdemo.ino"
#SKETCH := "examples/sourcedemo/sourcedemo.ino"
SKETCH := "examples/interruptsinkdemo/interruptsinkdemo.ino"

build:
	arduino-cli compile -b {{ BOARD }} {{ SKETCH }}

program: build
	arduino-cli upload -p {{ SERIAL_DEV }} -b {{ BOARD }} {{ SKETCH }}

run: program
	@# Wait for the file to go away
	while [[ -c {{ SERIAL_DEV }} ]]; do sleep 0.1s; done

	@# Then wait for it to show up again
	while ! [[ -c {{ SERIAL_DEV }} ]]; do sleep 0.1s; done

	@# Then finally wait for udev to change its permissions.
	while [[ $(stat -Lc "%G" {{ SERIAL_DEV }}) != "dialout" ]]; do sleep 0.1s; done

	arduino-cli monitor -p {{ SERIAL_DEV }}
