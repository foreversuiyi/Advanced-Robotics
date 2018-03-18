r = 0:0.01:5;
a = 2./(r+1./r);
plot(r,a,'k-')
xlabel('r*');
ylabel('a*');
set(gcf, 'color', 'white');