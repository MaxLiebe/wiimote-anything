CC ?= gcc
CFLAGS ?= -std=gnu11 -Wall -Wextra -Werror -g
INCLUDES := -I../include -I../src

# The backend test compiles against BTstack's headers. Default: the copy that
# Bluepad32 vendors next to this library (components/bluepad32).
BTSTACK_ROOT ?= ../components/bluepad32/external/btstack
BT_INCLUDES := -Ihost -I$(BTSTACK_ROOT)/src -I$(BTSTACK_ROOT)/src/classic \
	-I$(BTSTACK_ROOT)/3rd-party/bluedroid/encoder/include \
	-I$(BTSTACK_ROOT)/3rd-party/bluedroid/decoder/include \
	-I$(BTSTACK_ROOT)/3rd-party/yxml

PROTO_SRCS := ../src/wiimote_crypto.c ../src/wiimote_ext.c ../src/wiimote_proto.c

.PHONY: test proto backend clean

test: proto backend

proto: test_proto
	./test_proto

backend:
	@if [ -d "$(BTSTACK_ROOT)/src" ]; then \
		$(MAKE) --no-print-directory test_btstack_backend && ./test_btstack_backend; \
	else \
		echo "skipping backend test: BTstack not found at $(BTSTACK_ROOT) (set BTSTACK_ROOT=...)"; \
	fi

test_proto: test_proto.c $(PROTO_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ test_proto.c $(PROTO_SRCS)

test_btstack_backend: test_btstack_backend.c ../src/wiimote_bt_btstack.inc $(PROTO_SRCS)
	$(CC) -std=gnu11 -Wall -Wextra -g $(BT_INCLUDES) $(INCLUDES) -o $@ test_btstack_backend.c $(PROTO_SRCS) \
		$(BTSTACK_ROOT)/src/btstack_util.c $(BTSTACK_ROOT)/src/classic/sdp_util.c

clean:
	rm -f test_proto test_btstack_backend
