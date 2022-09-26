clear all;
close all;
clc;

I_rgb = imread('input_images/vegetables.jpeg');
I = rgb2gray(I_rgb);
[I_sx,I_sy] = size(I);


th = 0.2;

I_edge = edge(I,'sobel',th);

imwrite(I_edge,"output_images/image_sobel_edge.jpeg");