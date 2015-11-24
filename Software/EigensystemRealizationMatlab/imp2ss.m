function [a,b,c,d,totbnd,hsv] = imp2ss(Z1,Z2,Z3,Z4,Z5)
%IMP2SS System identification via impulse response (Kung's SVD algorithm).
% [A,B,C,D,TOTBND,HSV] = IMP2SS(Y) or
% [A,B,C,D,TOTBND,HSV] = IMP2SS(Y,TS,NU,NY,TOL) or
% [SS_,TOTBND,HSV] = IMP2SS(IMP_) 
% [SS_,TOTBND,HSV] = IMP2SS(IMP_,TOL) produces an approximate 
% state-space  realization of a given impulse response 
%                 IMP_=MKSYS(Y,TS,NU,NY,'imp')
% using the Hankel SVD method proposed by S. Kung (Proc. Asilomar
% Conf. on Circ. Syst. & Computers, 1978).  A continuous-time 
% realization is computed via the the inverse Tustin transform if
% TS is positive; otherwise, a discrete-time realization is returned.
%  INPUTS:  Y --- impulse response H1,...,HN stored rowwise
%                 Y=[H1(:)'; H2(:)'; H3(:)'; ...; HN(:)']
%  OPTIONAL INPUTS:
%           TS  ---  sampling interval (default  TS=0 --- discrete-time)
%           NU  ---  number of inputs  (default NU=1)
%           NY  ---  number of outputs (default NY= size(Y)*[1;0]/nu)
%           TOL ---  Hinfnorm of error bound (default TOL=0.01*S(1))
%  OUTPUTS: (A,B,C,D) state-space realization
%           TOTBND  Infinity norm error bound  2*Sum([S(NX+1),S(NX+2),...])
%           HSV  Hankel singular values [S(1);S(2);... ]
% R. Y. Chiang & M. G. Safonov 8/91
% Copyright (c) 1991-94 by the MathWorks, Inc.
% All Rights Reserved.
nag1 = nargin ;
nag2 = nargout ;
inargs = '(y,ts,nu,ny,tol)' ;
eval(mkargs(inargs,nargin,'imp'))
y = y' ;
[mm,nn] = size(y) ;
if ( nn == 1 )
    y = y' ;
    [mm,nn] = size(y) ;
end
% Do defaults for TS, NU, NY and TOL.
if ( nargin < 2 )
    ts = 0 ;
end
if ( nargin < 3 )
    nu = 1 ;
end
if ( nargin < 4 )
    if ( rem(mm,nu) ~= 0 )
        error('Incompatible Dimensions---The row dimension of Y must be divisible by NU') ;
    end
    ny = mm/nu ;
end
% Check dimensional compatibility.
if ( ny*nu ~= mm )
    error('Incompatable Dimensions---Must have NY*NU = (no. of columns of Y)') ;
end
z = zeros(ny,nu) ;
% Get D matrix D = H(0).
d = z ;
d(:) = y(:,1) ;
% Overwrite Y with Y = [ H1 H2 ... Hnn ].
tmp = y ;
m = ny ;
n = nn*nu ;
y = zeros(m,n) ;
y(:) = tmp(:) ;
% Build Hankel matrix H = H(1)  H(2) ... H(nn)
%                         H(2)  H(3) ... 0
%                         :     :        :
%                         H(nn) 0    ... 0
if ( ( nu == 1 ) && ( ny == 1 ) )
    h = hankel(y(2:nn)) ;
else
  for i=1:nn
    y=[y(:,nu+1:n) z];   % y= [H(i), H(i+1),...,H(nn),0,...,0]
    h=[h;y];
  end    
end
[rh,ch]=size(h);
% Compute the SVD of the Hankel
[u,hsv,v] = svd(h);
hsv=diag(hsv);
ns=size(hsv);
if nargin<5,tol=0.01*hsv(1);end
% Determine NX and TOTBND
tail=[conv(ones(ns,1)',hsv') 0];
tail=tail(ns:ns+ns);
nx=find(2*tail<=tol*ones(tail));
nx=nx(1)-1;
totbnd=2*tail(nx+1);  % TOTBND = 2*(HSV(NX+1)+HSV(NX+2)+...+HSV(NS))
% Truncate and partition singular vector matrices U,V
u1 = u(1:rh-ny,1:nx);
v1 = v(1:ch-nu,1:nx);
u2 = u(ny+1:rh,1:nx);
u3 = u(rh-ny+1:rh,1:nx);
ss = sqrt(hsv(1:nx));
invss = ones(nx,1)./ss;
% Kung's formula for the reduced model:
% The following is equivalent to UBAR = PINV(U)*[U2; 0]:
ubar = u1'*u2;
a = ubar.*(invss*ss');   % A = INV(DIAG(SS))*UBAR*DIAG(SS)
b = (ss*ones(1,nu)).*v1(1:nu,:)';  % B = DIAG(SS)*V1(1:NU,:)'
c = u1(1:ny,:).*(ones(ny,1)*ss');  % C = U1(1:NY,:)*DIAG(SS)
if ts>0,
  [a,b,c,d]=bilin(a,b,c,d,-1,'Tustin',ts); % convert to continuous
end  
if nag2<4;
   ss_ = mksys(a,b,c,d);
   a = ss_;
   b=totbnd;
   c=hsv;
end
% -------- End of IMP2SS.M % RYC/MGS
end
