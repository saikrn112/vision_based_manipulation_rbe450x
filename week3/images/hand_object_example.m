clear all;
close all;
clc;

I_rgb = imread('input_images/hand_object.png');
I = rgb2gray(I_rgb);
[I_sx,I_sy] = size(I);


th = 0.25;

I_edge = edge(I,'sobel',th);

imwrite(I_edge,"output_images/image_object.jpeg");