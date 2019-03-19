flash : thermometer.bin
	particle flash Temp1 thermometer.bin

thermometer.bin : src/thermometer.ino
	particle compile photon . --saveTo thermometer.bin

print : src/thermometer.ino
	vim -c 'hardcopy > output.ps' -c quit src/thermometer.ino && ps2pdf output.ps >output.pdf
