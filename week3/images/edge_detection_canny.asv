clear all;
close all;
clc;

I_rgb = imread('input_images/vegetables.jpeg');
I = rgb2gray(I_rgb);
[I_sx,I_sy] = size(I);

th_h_high = 0.21;
th_h_low = 0.2;

I_edge = edge(I,'canny',[th_h_low,th_h_high]);

imwrite(I_edge,"output_images/image_canny_edge.jpeg");