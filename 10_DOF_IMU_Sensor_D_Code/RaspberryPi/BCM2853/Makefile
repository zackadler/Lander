10Dof-D:main.o Waveshare_10Dof-D.o
	gcc -Wall -o 10Dof-D main.o Waveshare_10Dof-D.o -lbcm2835 -lm -std=gnu99
main.o: main.c Waveshare_10Dof-D.h
	gcc -Wall -c main.c -lbcm2835 -lm -std=gnu99
Waveshare_10Dof-D.o: Waveshare_10Dof-D.c Waveshare_10Dof-D.h
	gcc -Wall -c Waveshare_10Dof-D.c -lbcm2835 -lm -std=gnu99	
clean:
	rm main.o Waveshare_10Dof-D.o 10Dof-D