function [x,y,vib,dis]=visibilitytesting(Tx,Ty,Tz,w2,p2,k2,P,f,Ro,No)
 
vib=zeros(1,8);
 if size(P,2)>3
     P(:,4)=[];
 end
 PP=[P,(1:size(P,1))'];
 size(Tx,1)
for j=1:1: size(Tx,1) %%% all cameras
    pp=[];xa=[];ya=[];a=[];
C= [Tx(j,1) ,Ty(j,1),Tz(j,1)] ;
visiblePtInds=HPR(PP(:,1:3),C,Ro) ; %% hidden point removal
Pk=PP(visiblePtInds,:) ;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  pp=[0 0 0]; 
mw=[1 0 0;0 cos(w2(j,1)) sin(w2(j,1)) ;0 -sin(w2(j,1)) cos(w2(j,1))];
mp=[cos(p2(j,1)) 0 -sin(p2(j,1));0 1 0;sin(p2(j,1)) 0 cos(p2(j,1))];
mk=[cos(k2(j,1)) sin(k2(j,1)) 0;-sin(k2(j,1)) cos(k2(j,1)) 0;0 0 1];
%   m90=[1 0 0;0 cos( pi/2) sin( pi/2);0 -sin( pi/2) cos( pi/2) ];
m=mk*mp*mw;% 
dx1 =Pk(:,1)-Tx(j,1);
dy1 =Pk(:,2)-Ty(j,1);
dz1 =Pk(:,3)-Tz(j,1);
q =m(3,1).*dx1 +m(3,2).*dy1 +m(3,3).*dz1 ;
r =m(1,1).*dx1 +m(1,2).*dy1 +m(1,3).*dz1 ;
s =m(2,1).*dx1 +m(2,2).*dy1 +m(2,3).*dz1 ;

    x =-f*(r./q);% i=poit;j=image
    y =-f*(s./q); 
a=find(x <11.4 &  x >-11.4 &  y <7.3 &  y > -7.3 );%% based on my camera format size canon 500D
pp=Pk(a,:);xa=x(a,:);ya=y(a,:);
% format short g
vib=[vib; No(j,1)*ones(size(pp,1),1) pp(:,4) 2*ones(size(pp,1),1)  pp(:,1:3) xa  ya ];

  
D=sqrt(dx1.^2+dy1.^2+dz1.^2);dis(j,1)=mean(D);
%  figure  
%    plot3(C(:,1), C(:,2) ,C(:,3),'ko');hold on
%    plot3(P(:,1), P(:,2) ,P(:,3),'b.');hold on
%    plot3(pp(:,1), pp(:,2) ,pp(:,3),'r*');hold on
% text(C(:,1), C(:,2) ,C(:,3),num2str(j));hold on
% axis image
% axis off
% hold on
 end
 vib(1,:)=[];

end
 

    
 
  
