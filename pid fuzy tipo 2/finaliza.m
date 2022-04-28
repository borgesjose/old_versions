global SerPIC
flushoutput(SerPIC);
fclose(SerPIC); %--close the serial port when done
delete(SerPIC);
delete(instrfind);
%clear all;
clc;
