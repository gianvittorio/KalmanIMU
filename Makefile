CC = gcc
CFLAGS = -g -Wall
OBJS = example.o KalmanIMU.o
TARGET = example
#INCLUDES = ...
LIBS = -lm

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LIBS)
	
clean:
	rm -f $(TARGET) $(OBJS) *~