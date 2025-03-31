# Compiler and flags
CC = g++ -std=c++17
CFLAGS = -Wall -pthread

# Target executable
TARGET = mcp2515_test

# Source files
SRCS = mcp2515_test.cpp can_communication.cpp ../common/mcp2515_common.c

#Default target
all: $(TARGET)

# Linke the executable
$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS)

# Clean target
clean:
	rm -f $(TARGET)
	rm -f *.o