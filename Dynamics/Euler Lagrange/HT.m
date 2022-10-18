function T=HT(M,axis,value)
    if M=='R'
        if axis=='x'
            T=[1,0,0,0;0,cos(value),-sin(value),0;0,sin(value),cos(value),0;0,0,0,1];
        end
        if axis=='y'
            T=[cos(value),0,sin(value),0;0,1,0,0;-sin(value),0,cos(value),0;0,0,0,1];
        end
        if axis=='z'
            T=[cos(value),-sin(value),0,0;sin(value),cos(value),0,0;0,0,1,0;0,0,0,1];
        end
    else        
        if axis=='x'
            T=[1,0,0,value;0,1,0,0;0,0,1,0;0,0,0,1];
        end
        if axis=='y'
            T=[1,0,0,0;0,1,0,value;0,0,1,0;0,0,0,1];
        end
        if axis=='z'
           T=[1,0,0,0;0,1,0,0;0,0,1,value;0,0,0,1];
        end
    end
end