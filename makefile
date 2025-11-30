

.PHONY: all
all:
	pio run

.PHONY: upload
upload:
	pio run --target upload

.PHONY: clean
clean:
	pio run --target clean

.PHONY: fullclean
fullclean:
	pio run --target fullclean
