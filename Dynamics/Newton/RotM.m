function T=RotM(axis,value)
        if axis=='x'
            T=[1,0,0;0,cos(value),-sin(value);0,sin(value),cos(value)];
        end
        if axis=='y'
            T=[cos(value),0,sin(value);0,1,0;-sin(value),0,cos(value)];
        end
        if axis=='z'
            T=[cos(value),-sin(value),0;sin(value),cos(value),0;0,0,1];
        end
end