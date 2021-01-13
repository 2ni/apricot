# gcc -DF_CPU=10000000 -std=c++11 -I mocks -I ../common ../common/pins.cpp ../common/cmac.cpp ../common/aes.cpp ../common/lorawan.cpp ../common/rfm95.cpp spi.cpp sleep.cpp uart.cpp millis.cpp delay.cpp test.cpp main.cpp -o test && ./test

PRJ        = test
CLK        = 10000000

SRC        = .
COMMONDIR  = ../common
COMMON     = pins.cpp cmac.cpp aes.cpp lorawan.cpp rfm95.cpp
MOCKS      = ./mocks

FLAGS      = -std=c++11
CPPFLAGS   =

CFLAGS     = -DF_CPU=$(CLK) $(FLAGS) -I $(MOCKS) -I $(COMMONDIR)

# ********************************************************

CC         = gcc

# CFILES    := $(foreach dir, $(COMMON) $(SRC), $(wildcard $(dir)/*.c))
# CPPFILES  := $(foreach dir, $(COMMON) $(SRC), $(wildcard $(dir)/*.cpp))
CPPFILES  := $(foreach dir, $(SRC), $(wildcard $(dir)/*.cpp))

# objects from c files
all:
	@$(CC) $(CFLAGS) -o $(PRJ) $(patsubst %, ../common/%, $(COMMON)) $(CPPFILES) && ./$(PRJ)
