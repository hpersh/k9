CC	= gcc -g

all:
	$(CC) -c -Wa,-ahls k9_i386.s
	$(CC) -c -D__UNIT_TEST__ k9.c
	$(CC) -o k9 k9.o k9_i386.o

clean:
	rm -f *~ *.o k9
