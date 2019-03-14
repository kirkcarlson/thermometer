flash : MultipleThermometer.bin
	particle flash oled1 MultipleThermometer.bin

MultipleThermometer.bin : src/MultipleThermometer.ino
	particle compile photon . --saveTo MultipleThermometer.bin

print : src/MultipleThermometer.ino
	vim -c 'hardcopy > output.ps' -c quit src/MultipleThermometer.ino && ps2pdf output.ps >output.pdf
