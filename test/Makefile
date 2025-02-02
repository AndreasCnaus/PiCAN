# Compiler and flags
CC = g++
CFLAGS = -Wall -pthread

# Target executable
TARGET = mcp2515_test

# Source files
SRCS = mcp2515_test.cpp

#Default target
all: $(TARGET)

# Linke the executable
$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS)

# Clean target
clean:
	rm -f $(TARGET)
	rm -f *.o