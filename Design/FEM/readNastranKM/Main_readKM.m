% This code was developed to read stiffness and mass matrix exported by
% NASTRAN from punch file .pch through SOL 101
% =========================================================================
% after begin bulk
% PARAM,COUPLMASS,1
% PARAM,EXTOUT,DMIGPCH
% =========================================================================
% Copyright (c) 2015, Wei Zhao (weizhao@vt.edu)
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
% * Redistributions of source code must retain the above copyright notice, this
%   list of conditions and the following disclaimer.
% 
% * Redistributions in binary form must reproduce the above copyright notice,
%   this list of conditions and the following disclaimer in the documentation
%   and/or other materials provided with the distribution.
% 
% * Neither the name of the {organization} nor the names of its
%   contributors may be used to endorse or promote products derived from
%   this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

clear all;clc
% componment in GRDSET
GRDSET=[1 2 6];
componentAset=setdiff(1:6,GRDSET);
% node label in FEM
nodalabel=[1:16];
% leave SPCnodalabel blank for free-free vibration analysis
SPCnodalabel=[];
%===============
Asetnodalabel=setdiff(nodalabel,SPCnodalabel);
% nodalabel=1:10
%
[filename,filepath]=uigetfile([ '*.pch'],'Select Nastran Output Stiffness and Mass Matrix, .pch');
pchfname=[filepath  filename];
%
[Astiffness,Amass]=read_nastranKM(pchfname,componentAset,Asetnodalabel);
%
% 
for ii=1:size(Astiffness,2)
    
    for jj=ii:size(Astiffness,2)
        
        Astiffness(ii,jj)=Astiffness(jj,ii);
        Amass(ii,jj)=Amass(jj,ii);
    end
    
end
%
%
%% Eigenvalue computation, suitable for small order matrix and mode shape plot
[modeshape,eigvalue] = eig(Astiffness,Amass)
%
% for ee=2%1:size(modeshape,2)
% %   
%    figure
%    plot(-2:2,modeshape(1:2:9,ee))
% %   
% end
