a = arduino();


buttonState = 0;
counter = 0;

while ~buttonState
% Clear the connection

    if ~readDigitalPin(a, 'D2') 
        buttonState = 1;
    end
    counter = counter + 1;
    x = readVoltage(a, 'A1');
    y = readVoltage(a, 'A0');
    
    fprintf("Step: %d | X: %d | Y: %d | SEL: %d \n", counter, x , y, buttonState);
end

clear a;

