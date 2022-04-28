function z = inicializa(porta)
global SerPIC
SerPIC = serial(porta); %<--change this appropriately
set(SerPIC,'BaudRate', 9600, 'DataBits', 8, 'Parity', 'none','StopBits', 1, 'FlowControl', 'none');
fopen(SerPIC); %--open the serial port to the PIC