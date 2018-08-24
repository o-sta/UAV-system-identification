close all;
num = [1]
den = [4 5 6]

cP = tf(num,den)

[y,t] = impulse(cP);
[y,t] = step(cP);
figure(1)
hold on
plot(t,y)


dP = c2d(cP,0.2,'tustin')
[y,t] = impulse(dP);
[y,t] = step(dP);
stem(t,y)

cP_est = d2c(dP,'tustin')
[y,t] = step(cP);
plot(t,y)


