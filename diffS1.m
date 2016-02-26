function  out=diffS1(a,b)
% compute the difference of two angles in S1
% the result is the representation of the difference with least absolute
% value

res1=mod(a-b,2*pi); %result in [0,2*pi]
res2=res1-2*pi;

if(abs(res1)<abs(res2))
    out=res1;
else
    out=res2;
end