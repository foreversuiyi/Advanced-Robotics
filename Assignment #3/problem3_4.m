k = [1.25, 1.5, 1.75];
r0 = 0:0.01:5;
figure(1)
set(gcf, 'color', 'white');
hold on
for i = 1:length(k)
    r = k(i)*r0;
    a(i,:) = 2./(r+1./r0);
end
plot(r0, a(1,:), 'k-');
plot(r0, a(2,:), 'k--');
plot(r0, a(3,:), 'k-.');
xlabel('r*');
ylabel('a*');
legend('k = 1.25', 'k = 1.5', 'k = 1.75')