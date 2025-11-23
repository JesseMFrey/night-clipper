

.PHONY: all
all:
	pio run

.PHONY: upload
upload:
	pio run --target upload
