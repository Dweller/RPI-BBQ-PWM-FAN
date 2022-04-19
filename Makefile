CC      = gcc
CFLAGS  = -g
RM      = rm -f
LIBS    = -lmicrohttpd -lwiringPi -lpthread


default: all

all: bbq

bbq: bbq.c
	$(CC) $(CFLAGS) -o bbq bbq.c MAX6675.c $(LIBS)

clean:
	$(RM) bbq bbq.o
