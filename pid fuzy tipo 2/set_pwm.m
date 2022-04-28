function x = set_pwm(duty)
global SerPIC
fprintf(SerPIC,'%c','p') ;
duty = int16((250)*(duty/100));
dutyy = num2str(duty);
fprintf(SerPIC, '%s\r', dutyy);
end