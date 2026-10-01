function drawMaskCore(g,W,Hh,xx,mk,H,col){
var L=34,R=8,T=8,B=22,pw=W-L-R,ph=Hh-T-B;function X(f){return L+(f+170)/340*pw;}function Y(d){return T+(-d)/80*ph;}
function dec(s){var o=[];for(var i=0;i<s.length;i++){var k=s.charCodeAt(i);o.push(-1.25*(k>=65&&k<=90?k-65:k>=97&&k<=122?k-97+26:k-48+52));}return o;}
function mask(f){f=Math.abs(f);return f<=74?0:f<=107.5?-15*(f-74)/33.5:f<=124?-15-15*(f-107.5)/16.5:f<=152.5?-30-10*(f-124)/28.5:-40;}
function green(t){var a=[10,52,26],b=[52,199,89];return 'rgb('+[0,1,2].map(function(k){return Math.round(a[k]+(b[k]-a[k])*t);}).join(',')+')';}
function over(v,i){return v[i]>mask((i-28)*6)+0.05;}
g.clearRect(0,0,W,Hh);g.lineWidth=1;g.strokeStyle=col.sep;g.fillStyle=col.tx2;g.font='10px system-ui';g.textAlign='right';
for(var q=0;q>=-80;q-=20){g.beginPath();g.moveTo(L,Y(q));g.lineTo(L+pw,Y(q));g.stroke();g.fillText(q,L-4,Y(q)+3);}
g.textAlign='center';for(var f=-150;f<=150;f+=50)g.fillText(f,X(f),Hh-8);
var n=H.length;
for(var h=0;h<n;h++){var v=dec(H[h]);g.fillStyle=green(n>1?h/(n-1):1);g.beginPath();g.moveTo(X(-168),Y(-80));v.forEach(function(d,i){g.lineTo(X((i-28)*6),Y(d));});g.lineTo(X(168),Y(-80));g.closePath();g.fill();}
var v=dec(H[n-1]);
for(var i=0;i<v.length;i++){if(!over(v,i))continue;var a=i;while(i<v.length&&over(v,i))i++;var b=i-1,lo=Math.max(0,a-1),hi=Math.min(v.length-1,b+1);
g.fillStyle='#ff3b30';g.beginPath();for(var k=lo;k<=hi;k++){var fk=(k-28)*6;if(k==lo)g.moveTo(X(fk),Y(Math.max(v[k],mask(fk))));else g.lineTo(X(fk),Y(Math.max(v[k],mask(fk))));}
for(var k=hi;k>=lo;k--){var fk=(k-28)*6;g.lineTo(X(fk),Y(mask(fk)));}g.closePath();g.fill();}
if(xx.nf&&xx.nf.length>=57){g.strokeStyle=col.tx2;g.lineWidth=1;g.setLineDash([2,3]);g.beginPath();dec(xx.nf).forEach(function(d,i){if(i)g.lineTo(X((i-28)*6),Y(d));else g.moveTo(X((i-28)*6),Y(d));});g.stroke();g.setLineDash([]);}
var M=[[-170,-40],[-152.5,-40],[-124,-30],[-107.5,-15],[-74,0],[74,0],[107.5,-15],[124,-30],[152.5,-40],[170,-40]];
g.strokeStyle=col.tx;g.lineWidth=1.5;g.setLineDash([5,3]);g.beginPath();M.forEach(function(p,i){if(i)g.lineTo(X(p[0]),Y(p[1]));else g.moveTo(X(p[0]),Y(p[1]));});g.stroke();g.setLineDash([]);
if(xx.sp&&xx.sp.length>=57){var m=dec(xx.sp);g.lineWidth=2;for(var i=1;i<m.length;i++){g.strokeStyle=(over(m,i)||over(m,i-1))?'#ff3b30':col.tx;g.beginPath();g.moveTo(X((i-29)*6),Y(m[i-1]));g.lineTo(X((i-28)*6),Y(m[i]));g.stroke();}}
if(n>1){var bx=W-R-86,gr=g.createLinearGradient(bx,0,bx+60,0);gr.addColorStop(0,green(0));gr.addColorStop(1,green(1));g.fillStyle=gr;g.fillRect(bx,T+2,60,6);g.fillStyle=col.tx2;g.textAlign='right';g.fillText('older',bx-3,T+8);g.textAlign='left';g.fillText('newer',bx+63,T+8);}
}
