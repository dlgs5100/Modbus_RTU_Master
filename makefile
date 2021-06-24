CC := gcc
CFLAGS := -g -w
EXE := modbus
OBJS := main.o rtu_master.o
MATH := -lm

modbus: $(OBJS)
	$(CC) $(CFLAGS) -o $(EXE) $(OBJS) $(MATH)
main.o: main.c
	$(CC) $(CFLAGS) -c main.c $(MATH)
rtu_master.o: rtu_master.c rtu_master.h
	$(CC) $(CFLAGS) -c rtu_master.c rtu_master.h
clean:
	rm -f $(EXE) $(OBJS) *.h.gch 