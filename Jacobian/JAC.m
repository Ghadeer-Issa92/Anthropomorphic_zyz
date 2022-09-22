 clear;clc;
 %q = sym('q',[1 6]);
 %Jg =simplify(Jg(q)) 
 %Jn = simplify(Jn(q))
 %diff=Jn-Jg;
 %diff=simplify(diff)


%Testing singularity case
qq=[0 0 -pi/2 0 1 0];
h =Jg(qq)
rank(h)
[U,S,V]=svd(h)
rank(S)


