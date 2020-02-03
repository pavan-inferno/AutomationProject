clear all; clc; close all;
% The code allows any high contrast file to be read irrespective of its
% shape or dimensions
%im = imread('pressure blank.png');
im = imread('mattress.PNG');
subplot(2,2,1);
imshow(im)
%convert image to grayscale
bw = rgb2gray(im);
%Convert image to binary format as bwboundaries accpets ONLY binary images
bw_bin = im2bw(bw,0.7);
subplot(2,2,2);
imshow(bw)
%Inversion of image
bw_bin = ~bw_bin;
figure()
imshow(~bw_bin)
%Using the bwboundaries function in MATLAB, L is the contains cells that
%store vertices information and L is a label matrix that classifies element
%into object and background
[B,L] = bwboundaries(bw_bin,'noholes');
temp = regionprops(bw_bin,'Centroid');
centroids = vertcat(temp.Centroid);
% perm_numbers = 1:length(B);
%To reaarange the elements in the preferred order
perm_numbers = [7:20 22:91 1:6 21 96 95 94 93 92];
%Plotting of the image 

total_number = 96;
including_dummies = 70;

% control_number = [7:20 22:91 1:6 21 96 95 94 93 92];
control_number = 1:length(B);

for k = 1:length(B)
vertices{k} = B{perm_numbers(k)}; 
centers{k} = centroids(perm_numbers(k),:);
hold on; 
plot(vertices{k}(:,2), vertices{k}(:,1), 'r', 'LineWidth', 2); hold on
%plot(centroids(perm_numbers(k),1),centroids(perm_numbers(k),2),'*r');  hold on; 
% text(centroids(perm_numbers(k),1)-9,centroids(perm_numbers(k),2),num2str(control_number(k)),'FontSize',15,'Color','k');
text(centroids(perm_numbers(k),1)-9,centroids(perm_numbers(k),2),num2str(k),'FontSize',15,'Color','k');
end

xmax = -10000;
xmin = 10000;
ymax = -10000;
ymin = 10000;
figure()
for k = 1:length(B)
    poly1 = (vertices{k})';
    poly2 = reduce_poly(poly1,25);
    poly_draw = [poly2 poly2(:,1)];
    vertices{k} = poly_draw';
    plot(vertices{k}(:,2), vertices{k}(:,1), 'r', 'LineWidth', 2); hold on
    %plot(centroids(perm_numbers(k),1),centroids(perm_numbers(k),2),'*r');  hold on; 
    text(centroids(perm_numbers(k),1)-9,centroids(perm_numbers(k),2),num2str(control_number(k)),'FontSize',15);
    if control_number(k)<=total_number
        if max(vertices{k}(:,1))>ymax
            ymax = max(vertices{k}(:,1));
        end
        if min(vertices{k}(:,1))<ymin
            ymin = min(vertices{k}(:,1));
        end
        if max(vertices{k}(:,2))>xmax
            xmax = max(vertices{k}(:,2));
        end
        if min(vertices{k}(:,2))<xmin
            xmin = min(vertices{k}(:,2));
        end
    end
    
end
axis equal


InfoImage = imfinfo('mattress.PNG');
width = InfoImage.Width;
height = InfoImage.Height;

save('mattress_vertices.mat','vertices','centers','height','width','control_number','total_number','including_dummies','xmin','xmax','ymin','ymax');
hold on
plot(0,0,'bo');
hold on
plot(width,height,'b*');
plot(xmin,ymin,'b+');
plot(xmin,ymax,'b+');
plot(xmax,ymax,'b+');
plot(xmax,ymin,'b+');
