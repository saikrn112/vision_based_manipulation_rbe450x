close all;
clear all;
clc;

% Image threshdoling with Matlab function

I_color = imread("input_images/WPI_campus.jpeg");
th_val = 200/255;

I = rgb2gray(I_color);
I_th = im2bw(I, th_val);

imshow (I_th)

imwrite(I_th,"output_images/image_bw_th.jpeg");