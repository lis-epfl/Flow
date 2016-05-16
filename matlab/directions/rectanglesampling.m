width = 160;
height = 120;
nPoints = 125;

totalArea = width*height;
pointArea = totalArea/nPoints;
w = width/sqrt(nPoints);
h = height/sqrt(nPoints);

figure;
hold on;

x = [];
y = [];

for(i = w/2:w:width),
  for(j = h/2:h:height),
    y = [y; floor(i)];
    x = [x; floor(j)];
    plot(x, y, 'bo');
  end
end
hold off;

rectangle_sampling = [x, y];
dlmwrite('rectangle_sampling.csv', rectangle_sampling, 'delimiter', ',', 'precision', 3);