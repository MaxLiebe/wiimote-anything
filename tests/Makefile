CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror
INCLUDES := -I../include -I../src

.PHONY: test clean

test: test_proto
	./test_proto

test_proto: test_proto.c ../src/wiimote_crypto.c ../src/wiimote_ext.c ../src/wiimote_proto.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ test_proto.c ../src/wiimote_crypto.c ../src/wiimote_ext.c ../src/wiimote_proto.c

clean:
	rm -f test_proto
