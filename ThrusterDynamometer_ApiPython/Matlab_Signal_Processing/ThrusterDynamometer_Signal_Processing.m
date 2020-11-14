temp = readmatrix('temp.txt')

time = temp(:,1)'
temp1 = temp(:,2)'
temp2 = temp(:,3)'

plot(time, temp1, time, temp2)