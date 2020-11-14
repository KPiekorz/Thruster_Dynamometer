tenso = readmatrix('tenso.txt');
% shunt = readmatrix('shunt.txt')

% time = temp(:,1)'
% temp1 = temp(:,2)'
% temp2 = temp(:,3)'
% 

time_tenso = tenso(:,1)';
tenso_value = tenso(:,2)'; 
% 
% time_shunt = shunt(:,1)';
% shunt_value = shunt(:,2)';


% shunt_value = ((73.3*shunt_value)/4095) - 36.7; 

% shunt_value = lowpass(shunt_value, 0.00000000000000001);

% d = fdesign.lowpass('Fp,Fst,Ap,Ast',0.3,0.8,0.5,40,100);
% Hd = design(d,'equiripple');
% shunt_value = filter(Hd,shunt_value);


% % Y = fft(shunt_value);

% T = 0.01;
% Fs = 1/T;
% L = 444;
% tf = (0:L-1)*T;
% 
% P2 = abs(Y/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% 
% f = Fs*(0:(L/2))/L;
% figure(2);
% plot(f,P1) 
% title('Single-Sided Amplitude Spectrum of X(t)')
% xlabel('f (Hz)')
% ylabel('|P1(f)|')


D = movmean(tenso_value,1000);
figure(1);
% subplot(1,2,1);
plot(time_tenso, D, 'r--');
% ylim([0 3000]);

% subplot(1,2,2);
% plot(time_shunt, shunt_value)

