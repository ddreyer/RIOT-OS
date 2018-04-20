include ../Makefile.tests_common

BOARD = hamilton

# This test application is for the hamilton only
BOARD_WHITELIST := hamilton

# sensor setting
USEMODULE += saul_default

# RTT module setting
CFLAGS    += -DRTT_STDIO_DISABLE_STDIN

include $(RIOTBASE)/Makefile.include
