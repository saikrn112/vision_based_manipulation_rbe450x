clear all;
close all;
clc;

I = imread('input_images/building.jpeg');


C = corner(I,'Harris',500);
imshow(I)
hold on
plot(C(:,1),C(:,2),'r*');