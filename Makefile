flash-stm32duino:
	cp memory.stm32duino.x memory.x
	cargo objcopy --example blink -- -O binary blink.bin
	sudo dfu-util -D blink.bin -a 2

flash-orig:
	cp memory.orig.x memory.x
	cargo objcopy --example blink -- -O binary blink.bin
	dfu-util -D blink.bin -a 1
