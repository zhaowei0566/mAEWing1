function damp2(p, tsam)
  % assume a continuous system
  DIGITAL = 0;
  if(nargin < 1 || nargin > 2)
    error('damp(p,[ tsamp])');
  end
  if(isstruct(p))
    if (nargin ~= 1)
      error('damp: when p is a system, tsamp parameter is not allowed.');
    end
    [aa, b, c, d, t_samp] = sys2ss(p);
    DIGITAL = is_digital(p);
  else
    aa = p;
    if (nargin == 2)
        DIGITAL = 1;
        t_samp = tsam;
    end
  end
  if (DIGITAL && t_samp <= 0.0)
    error('damp: Sampling time tsam must not be <= 0.');
  end
  % all checks done.
  e = eig(aa) ;
  [ n , m ] = size(aa) ;
  if ( DIGITAL )
    fprintf(1,'  (discrete system with sampling time %f)\n' , t_samp ) ;
  end
  fprintf(1,'Freq. (Hz)   Damp. Ratio\n');
  for i = 1 : 1 : n
    pole = e(i) ;
    cpole = pole ;
    if (DIGITAL)
      cpole = log(pole) / t_samp ;
    end
    d0 = -cos( atan2( imag(cpole) , real(cpole) ) ) ;
    f0 = 0.5 / pi * abs(cpole) ;
    fprintf(1,'%12f  %10f\n' , f0 , d0 ) ;
  end
end
