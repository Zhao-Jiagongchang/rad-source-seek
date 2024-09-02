function [x,y] = node2Pose(n,Worldsize)
    Worldsize=Worldsize+1;
    quotient = floor(n / Worldsize);
    x=quotient;
    remainder = mod(n, Worldsize);
    if remainder == 0
        y=remainder;
        x=x-1;
    else
        y=50-remainder;
    end
end