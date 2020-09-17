function [a,b] = class_test()
    class_test.counter = 1;
    a = class_test.counter;
    class_test.counter = class_test.counter + 1;
    b = class_test.counter;
    disp(a);
    disp(b);
end